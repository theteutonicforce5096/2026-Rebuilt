from typing import Final

from commands2 import Subsystem
from phoenix6 import BaseStatusSignal, CANBus
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX
from wpilib import RobotBase, RobotState, SmartDashboard, Timer
from wpimath.filter import Debouncer

from subsystems.device_config import check_signal_status, configure_device


class Arm(Subsystem):
    """
    Pivots the intake wheel between its floor, shooting, and stowed positions.

    The arm runs closed-loop to a handful of preset positions using a fused CANcoder for absolute
    feedback. Because it can jam against the frame or an obstruction, a stall watch cuts power when
    a commanded arm both stops closing position error and stays loaded while doing so.
    """

    # Rotations from a target position within which the arm counts as having arrived
    _ARM_POSITION_TOLERANCE_ROT: Final = 0.001

    # Minimum position error at which a stall may be declared. Inside this band the closed-loop
    # output tapers off and the arm legitimately stops closing error, which must not read as a jam.
    # Set to twice the arrival tolerance so the band still covers the taper once static friction
    # parks the arm at the far edge of that tolerance.
    _STALL_POSITION_ERROR_MIN_ROT: Final = 0.01

    def __init__(
        self,
        canbus: CANBus,
        arm_id: int,
        arm_encoder_id: int,
        arm_configs: TalonFXConfiguration,
        arm_encoder_configs: CANcoderConfiguration,
        num_config_attempts: int,
        intake_position: float,
        stowed_position: float,
        shooting_position: float,
        stall_progress_threshold: float,
        stall_time_threshold: float,
        stall_grace_sec: float,
        stall_current_threshold_amps: float,
    ):
        """
        Initialize the arm using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param arm_id: CAN ID of the arm motor
        :type arm_id: int
        :param arm_encoder_id: CAN ID of the arm encoder
        :type arm_encoder_id: int
        :param arm_configs: Configs for the arm motor
        :type arm_configs: phoenix6.configs.TalonFXConfiguration
        :param arm_encoder_configs: Configs for the arm encoder
        :type arm_encoder_configs: phoenix6.configs.CANcoderConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param intake_position: Encoder position where the arm is down
        :type intake_position: float
        :param stowed_position: Encoder position where the arm is up
        :type stowed_position: float
        :param shooting_position: Encoder position where the arm is at an intermediate position
        :type shooting_position: float
        :param stall_progress_threshold: Rotations of position error the arm must close per
            detection window to count as moving
        :type stall_progress_threshold: float
        :param stall_time_threshold: Detection window length in seconds, which is also the time
            constant of the progress-rate filter
        :type stall_time_threshold: float
        :param stall_grace_sec: Time after a new setpoint during which no stall may be declared,
            covering the arm reversing momentum out of an interrupted move
        :type stall_grace_sec: float
        :param stall_current_threshold_amps: Stator current the arm must draw for one detection
            window, alongside stalled progress, before a jam is declared
        :type stall_current_threshold_amps: float
        """
        Subsystem.__init__(self)

        # Create motor and encoder
        self.motor = TalonFX(arm_id, canbus)
        self.encoder = CANcoder(arm_encoder_id, canbus)

        # Apply configs
        configure_device(self.motor, arm_configs, num_config_attempts)
        configure_device(self.encoder, arm_encoder_configs, num_config_attempts)

        if not RobotBase.isSimulation():
            # Trim every signal, then raise only the ones the stall and move-complete checks poll
            # each loop. The encoder is fused into the motor on-device, so its own frame is only
            # used for logging.
            check_signal_status(
                self.motor.optimize_bus_utilization(),
                "Arm bus optimization",
            )
            check_signal_status(
                self.encoder.optimize_bus_utilization(),
                "Arm encoder bus optimization",
            )
            check_signal_status(
                self.motor.get_position().set_update_frequency(100.0),
                "Arm position update rate",
            )
            check_signal_status(
                self.motor.get_velocity().set_update_frequency(100.0),
                "Arm velocity update rate",
            )
            # Stall detection gates its power cut on this signal, so it runs at the same rate as
            # the position and velocity it is judged alongside. Publishing faster than the loop
            # reads guarantees a fresh sample every loop instead of one repeated across two.
            check_signal_status(
                self.motor.get_stator_current().set_update_frequency(100.0),
                "Arm stator current update rate",
            )
            check_signal_status(
                self.encoder.get_position().set_update_frequency(100.0),
                "Arm encoder position update rate",
            )

        # Create control requests. Closed-loop moves and the stall watch's power cut each get their
        # own request so one can never carry the other's stale output.
        self.voltage_request = VoltageOut(output=0)
        self.position_voltage_request = PositionVoltage(position=0)

        # Arm positions
        self.intake_position = intake_position
        self.stowed_position = stowed_position
        self.shooting_position = shooting_position

        # The presets ordered bottom to top, and where the arm sits on that ladder. Stepping walks
        # this index instead of comparing measured positions, so a press during a move steps from
        # the preset already commanded rather than from a position the arm is still travelling
        # through. An arm that has never been commanded starts at the bottom because an unpowered
        # arm sags there under gravity.
        self._presets = (intake_position, shooting_position, stowed_position)
        self._preset_index = 0

        # None until the arm has been commanded somewhere, which the stall and move-complete checks
        # both treat as "nothing to watch"
        self.commanded_position: float | None = None
        self.is_stalled = False

        # Stall detection tunables. The progress and time thresholds are kept in their per-window
        # form because that is how they are reasoned about (rotations closed per window), and the
        # rate the filter is actually compared against is derived from them once here.
        self.stall_progress_threshold = stall_progress_threshold
        self.stall_time_threshold = stall_time_threshold
        self.stall_grace_sec = stall_grace_sec
        self._stall_rate_threshold = stall_progress_threshold / stall_time_threshold

        # Low-passed rate at which the arm is closing position error, in rotations per second. The
        # filter's time constant is the detection window, so the verdict reflects roughly a
        # window's worth of history no matter how often it is updated. Judging a filtered rate
        # rather than instantaneous velocity tolerates the arm's noisy velocity signal at its slow
        # travel speed, while a signed rate still catches an arm being pushed backward.
        self._progress_rate = self._stall_rate_threshold
        self._last_error: float | None = None
        self._last_sample_time = 0.0
        self._stall_grace_timer = Timer()

        # A jam loads the motor, so stalled progress only counts as one when the arm is also
        # pulling current. A rising debouncer states that requirement directly: its output stays
        # false until the current has been over the threshold for a full detection window.
        self.stall_current_threshold_amps = stall_current_threshold_amps
        self._stall_current_debouncer = Debouncer(
            stall_time_threshold, Debouncer.DebounceType.kRising
        )
        self._is_loaded = False

        # Set once the arm reaches its commanded position, standing the watch down until the next
        # setpoint. The watch exists to protect moves; once the arm is holding, a disturbance that
        # pushes it off target is the current limit's job rather than a reason to cut power.
        self._has_arrived = False

        # Held rather than re-fetched so all three can be refreshed in one synchronized call, which
        # keeps the stall verdict on a single coherent sample of the mechanism.
        self._position_signal = self.motor.get_position()
        self._velocity_signal = self.motor.get_velocity()
        self._stator_current_signal = self.motor.get_stator_current()

        # Signals refreshed each loop for stall detection and telemetry
        self.stator_current = 0.0
        self.velocity = 0.0
        self.position = 0.0

    def periodic(self):
        """Refresh the arm signals used for stall detection and report them to the dashboard."""
        # One synchronized refresh so progress and load are judged on the same instant rather than
        # on three signals sampled at whatever point each was last polled.
        BaseStatusSignal.refresh_all(
            self._position_signal, self._velocity_signal, self._stator_current_signal
        )

        # Current is compared as a level against a threshold that must hold for a full window, so
        # a frame a few milliseconds old shifts the verdict by that much and nothing more. There is
        # no rate-of-change-of-current signal to compensate against in any case, which is why the
        # answer for this one is a fast frame rather than extrapolation.
        self.stator_current = self._stator_current_signal.value_as_double
        self.velocity = abs(self._velocity_signal.value_as_double)

        # Progress is a difference of positions, so a position frame that is a few milliseconds
        # stale shows up directly as rate noise. Compensating by the signal's own measured latency
        # removes that at the source; during a real jam the velocity term is zero, so the
        # correction vanishes exactly when it must not mask anything.
        self.position = BaseStatusSignal.get_latency_compensated_value(
            self._position_signal, self._velocity_signal
        )

        # The stall watch runs every loop, not just while an arm-move command is scheduled, so a
        # jam still cuts power even when nothing else is polling it. It runs before is_stalled is
        # published so the dashboard shows this loop's verdict.
        self.get_stall_detection()

        # Position and velocity are the signals the stall logic runs on, and current shows how hard
        # the arm is working; watching the real values is what makes the stall thresholds and
        # current limit tunable from measurements instead of guesses.
        SmartDashboard.putBoolean("Arm/Stalled", self.is_stalled)
        SmartDashboard.putBoolean("Arm/Loaded", self._is_loaded)
        SmartDashboard.putNumber("Arm/Current (A)", self.stator_current)
        SmartDashboard.putNumber("Arm/Velocity (rps)", self.velocity)
        SmartDashboard.putNumber("Arm/Position (rot)", self.position)
        SmartDashboard.putNumber("Arm/Progress Rate (rps)", self._progress_rate)

        # NetworkTables has no way to say "no value", so an idle arm reports -1 instead of a
        # position it was never commanded to.
        SmartDashboard.putNumber(
            "Arm/Commanded Position (rot)",
            self.commanded_position if self.commanded_position is not None else -1.0,
        )

    def set_setpoint(self, position):
        """
        Command the arm to the requested closed-loop position.

        :param position: Desired arm position in mechanism rotations.
        :type position: float
        """
        self.commanded_position = position

        # Keep the step ladder pointing at whatever was just commanded, so a step after a direct
        # move continues from there. A target that is not a preset leaves the ladder alone.
        if position in self._presets:
            self._preset_index = self._presets.index(position)

        # A new move starts clean, so clear any stall and any arrival left over from the previous
        # move before commanding the arm again.
        self.is_stalled = False
        self._has_arrived = False
        self._reset_stall_watch()

        self.motor.set_control(self.position_voltage_request.with_position(position))

    def detect_arm_movement_completion(self):
        """
        Report whether the arm has finished its move.

        :returns: True once the arm reaches the commanded position or a stall is recorded.
        :rtype: bool
        """
        if self.commanded_position is None:
            return False
        return self._has_arrived or self.is_stalled

    def set_arm_voltage(self, voltage):
        """
        Drive the arm with an open-loop voltage.

        :param voltage: Voltage to apply to the arm motor.
        :type voltage: float
        """
        self.motor.set_control(self.voltage_request.with_output(voltage))

    def stop_and_latch_stall(self):
        """
        Cut arm power and record the arm as stalled.

        Used both by the stall watch and by the arm-move timeout, so a move that gives up leaves
        the arm in the same state a detected jam does rather than straining against the position
        request it is still holding.
        """
        self.set_arm_voltage(0)
        self.is_stalled = True

    def arm_down(self):
        """Move the arm down to the floor-pickup position."""
        self.set_setpoint(self.intake_position)

    def arm_up(self):
        """Move the arm up to the stowed position."""
        self.set_setpoint(self.stowed_position)

    def arm_down_intermediate(self):
        """Move the arm to the intermediate position used while shooting."""
        self.set_setpoint(self.shooting_position)

    def step_arm_up(self):
        """Move the arm up one preset position toward stowed."""
        self.set_setpoint(self._next_arm_position(moving_up=True))

    def step_arm_down(self):
        """Move the arm down one preset position toward intake."""
        self.set_setpoint(self._next_arm_position(moving_up=False))

    def _next_arm_position(self, moving_up: bool) -> float:
        """
        Return the preset one step above or below the one the arm was last commanded to.

        Clamping the index bounds the travel, so stepping up from stowed or down from intake
        returns that same end rather than running off the ladder.

        :param moving_up: True to step toward the stowed position, False toward intake.
        :type moving_up: bool
        :returns: Preset position one step in the requested direction.
        :rtype: float
        """
        step = 1 if moving_up else -1
        index = min(len(self._presets) - 1, max(0, self._preset_index + step))
        return self._presets[index]

    def get_stall_detection(self):
        """
        Watch the moving arm and cut power if it stops making progress under load.

        A commanded arm must close position error at least as fast as the configured progress
        threshold allows for over one detection window. The rate is low-passed with the detection
        window as its time constant and every term is scaled by the real elapsed time, so the
        threshold means the same thing regardless of how often this runs and a jam is caught about
        one window after it happens rather than up to two.

        Stalled progress on its own is not enough: the arm must also have been drawing at least the
        current threshold for a full window, since a jam necessarily loads the motor. Requiring
        both measurements to agree means neither a slow-coasting arm nor a momentarily flat
        position signal can cut power on its own.

        :returns: True once a stall has been detected, otherwise False.
        :rtype: bool
        """
        if self.commanded_position is None:
            return False

        # A stall latches until the next setpoint clears it, so a jammed arm stays stopped instead
        # of re-driving the moment it springs back.
        if self.is_stalled:
            return True

        # A disabled arm sags under gravity and closes no error, which is not a jam. Dropping the
        # sample history also keeps the gap across a disabled period out of the filter.
        if not RobotState.isEnabled():
            self._reset_stall_watch()
            return False

        position_error = abs(self.commanded_position - self.position)

        if position_error <= self._ARM_POSITION_TOLERANCE_ROT:
            self._has_arrived = True

        # Near the target the closed-loop output tapers and progress legitimately shrinks, so the
        # watch stands down and arrival detection takes over (see the class constants).
        if self._has_arrived or position_error <= self._STALL_POSITION_ERROR_MIN_ROT:
            self._reset_stall_watch()
            return False

        # Sampled on every armed loop, ahead of the returns below, so a dip under the threshold
        # restarts the hold rather than pausing it partway. Stator current is signed by motoring
        # versus regenerative braking rather than by travel direction, so an arm driving into a jam
        # reads positive whichever way it pushes and no absolute value is wanted here.
        self._is_loaded = self._stall_current_debouncer.calculate(
            self.stator_current >= self.stall_current_threshold_amps
        )

        now = Timer.getFPGATimestamp()

        # The first sample of a watched move only establishes the reference the next one differs
        # against.
        if self._last_error is None:
            self._last_error = position_error
            self._last_sample_time = now
            return False

        elapsed = now - self._last_sample_time
        if elapsed <= 0.0:
            return False

        progress_rate = (self._last_error - position_error) / elapsed
        self._progress_rate += (progress_rate - self._progress_rate) * min(
            1.0, elapsed / self.stall_time_threshold
        )
        self._last_error = position_error
        self._last_sample_time = now

        # A setpoint that reverses a move in flight makes the arm lose ground while its momentum
        # turns around, which reads exactly like a jam, so no verdict is issued until that is over.
        if not self._stall_grace_timer.hasElapsed(self.stall_grace_sec):
            return False

        # Both conditions have to hold over the same window: the arm has stopped closing error and
        # is loaded while doing so. Either alone is ordinary - a light arm can coast slowly, and a
        # loaded arm making progress is just a normal move against gravity.
        if self._progress_rate < self._stall_rate_threshold and self._is_loaded:
            self.stop_and_latch_stall()
            return True

        return False

    def _reset_stall_watch(self):
        """Drop the sample history and restart the grace period so the next watch starts fresh."""
        self._last_error = None
        self._last_sample_time = 0.0
        self._progress_rate = self._stall_rate_threshold
        self._stall_grace_timer.restart()

        # Debouncer has no reset of its own, and a false input is what clears it, so the current
        # hold only accumulates across loops where the watch is actually armed.
        self._is_loaded = False
        self._stall_current_debouncer.calculate(False)
