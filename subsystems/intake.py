from typing import Final

from commands2 import Subsystem
from phoenix6 import CANBus
from phoenix6.configs import CANcoderConfiguration, TalonFXConfiguration, TalonFXSConfiguration
from phoenix6.controls import PositionVoltage, VoltageOut
from phoenix6.hardware import CANcoder, TalonFX, TalonFXS
from wpilib import RobotBase, SmartDashboard, Timer

from subsystems.device_config import check_signal_status, configure_device


class Intake(Subsystem):
    """
    Picks balls up off the floor and holds the pivoting arm that carries the intake wheel.

    The arm runs closed-loop to a handful of preset positions using a fused CANcoder for absolute
    feedback. Because the arm can pinch against the frame, a stall watch cuts power when the arm
    draws current without moving.
    """

    # Rotations from a target position within which the arm counts as having arrived
    _ARM_POSITION_TOLERANCE_ROT: Final = 0.009

    def __init__(
        self,
        canbus: CANBus,
        intake_wheel_id: int,
        intake_arm_id: int,
        intake_arm_encoder_id: int,
        intake_wheel_configs: TalonFXSConfiguration,
        intake_arm_configs: TalonFXConfiguration,
        intake_arm_encoder_configs: CANcoderConfiguration,
        num_config_attempts: int,
        intake_position: float,
        stowed_position: float,
        shooting_position: float,
        stall_current_threshold: float,
        stall_velocity_threshold: float,
        stall_time_threshold: float,
        arm_movement_pathway_low: float,
        arm_movement_pathway_high: float,
        obstruction_current_threshold: float,
        obstruction_dead_band: float,
    ):
        """
        Initialize the intake using the specified constants.

        :param canbus: CANBus instance that electronics are on
        :type canbus: phoenix6.CANBus
        :param intake_wheel_id: CAN ID of the intake wheel
        :type intake_wheel_id: int
        :param intake_arm_id: CAN ID of the intake arm
        :type intake_arm_id: int
        :param intake_arm_encoder_id: CAN ID of the intake arm encoder
        :type intake_arm_encoder_id: int
        :param intake_wheel_configs: Configs for the intake wheel
        :type intake_wheel_configs: phoenix6.configs.TalonFXSConfiguration
        :param intake_arm_configs: Configs for the intake arm
        :type intake_arm_configs: phoenix6.configs.TalonFXConfiguration
        :param intake_arm_encoder_configs: Configs for the intake arm encoder
        :type intake_arm_encoder_configs: phoenix6.configs.CANcoderConfiguration
        :param num_config_attempts: Number of times to attempt to configure each device
        :type num_config_attempts: int
        :param intake_position: Encoder position where arm is down
        :type intake_position: float
        :param stowed_position: Encoder position where arm is up
        :type stowed_position: float
        :param shooting_position: Encoder position where the arm is at an intermediate position
        :type shooting_position: float
        :param stall_current_threshold: Minimum stator current required to detect a stall
        :type stall_current_threshold: float
        :param stall_velocity_threshold: Arm velocity below which the arm counts as stalled
        :type stall_velocity_threshold: float
        :param stall_time_threshold: Time the stall conditions must hold before a stall is
            declared
        :type stall_time_threshold: float
        :param arm_movement_pathway_low: Low position in the arm movement pathway
        :type arm_movement_pathway_low: float
        :param arm_movement_pathway_high: High position in the arm movement pathway
        :type arm_movement_pathway_high: float
        :param obstruction_current_threshold: Minimum stator current required to detect an
            obstruction
        :type obstruction_current_threshold: float
        :param obstruction_dead_band: Range around the shooting position where high current is
            expected, not an obstruction
        :type obstruction_dead_band: float
        """
        Subsystem.__init__(self)

        # Create motors
        self.intake_wheel = TalonFXS(intake_wheel_id, canbus)
        self.intake_arm = TalonFX(intake_arm_id, canbus)
        self.intake_arm_encoder = CANcoder(intake_arm_encoder_id, canbus)

        # Apply motor configs
        configure_device(self.intake_wheel, intake_wheel_configs, num_config_attempts)
        configure_device(self.intake_arm, intake_arm_configs, num_config_attempts)
        configure_device(self.intake_arm_encoder, intake_arm_encoder_configs, num_config_attempts)

        if not RobotBase.isSimulation():
            # Trim every signal, then raise only the arm signals the stall and move-complete
            # checks poll each loop. The encoder is fused into the arm on-device, so its own
            # frame is only used for logging.
            check_signal_status(
                self.intake_wheel.optimize_bus_utilization(),
                "Intake wheel bus optimization",
            )
            check_signal_status(
                self.intake_arm.optimize_bus_utilization(),
                "Intake arm bus optimization",
            )
            check_signal_status(
                self.intake_arm_encoder.optimize_bus_utilization(),
                "Intake arm encoder bus optimization",
            )
            check_signal_status(
                self.intake_arm.get_position().set_update_frequency(100.0),
                "Intake arm position update rate",
            )
            check_signal_status(
                self.intake_arm.get_velocity().set_update_frequency(100.0),
                "Intake arm velocity update rate",
            )
            check_signal_status(
                self.intake_arm.get_stator_current().set_update_frequency(100.0),
                "Intake arm stator current update rate",
            )
            check_signal_status(
                self.intake_arm_encoder.get_position().set_update_frequency(100.0),
                "Intake arm encoder position update rate",
            )

        # Create control requests. The wheel and the arm each get their own voltage request so
        # one mechanism's command can never carry the other's stale output.
        self.wheel_voltage_request = VoltageOut(output=0)
        self.arm_voltage_request = VoltageOut(output=0)
        self.position_voltage_request = PositionVoltage(position=0)

        # Arm positions
        self.intake_position = intake_position
        self.stowed_position = stowed_position
        self.shooting_position = shooting_position

        # Obstruction detection tunables
        self.arm_movement_pathway_low = arm_movement_pathway_low
        self.arm_movement_pathway_high = arm_movement_pathway_high
        self.obstruction_current_threshold = obstruction_current_threshold
        self.obstruction_dead_band = obstruction_dead_band

        # None until the arm has been commanded somewhere, which the stall and move-complete
        # checks both treat as "nothing to watch"
        self.commanded_position: float | None = None
        self.is_stalled = False

        # Stall detection tunables
        self.stall_current_threshold = stall_current_threshold
        self.stall_velocity_threshold = stall_velocity_threshold
        self.stall_time_threshold = stall_time_threshold
        self.stall_timer = Timer()

        # Arm signals refreshed each loop for stall and obstruction detection
        self.arm_stator_current = 0.0
        self.arm_velocity = 0.0
        self.arm_position = 0.0

    def periodic(self):
        """Refresh the arm signals used for stall detection and report them to the dashboard."""
        self.arm_stator_current = self.intake_arm.get_stator_current().value_as_double
        self.arm_velocity = abs(self.intake_arm.get_velocity().value_as_double)
        self.arm_position = self.intake_arm.get_position().value_as_double

        # The stall watch runs every loop, not just while an arm-move command is scheduled, so
        # a jam still cuts power even when nothing else is polling it. It runs before is_stalled
        # is published so the dashboard shows this loop's verdict.
        self.get_stall_detection()

        # These are the same three signals the stall logic runs on, so publishing them costs no
        # extra CAN traffic. Watching the real current and position is what makes the obstruction
        # and stall thresholds tunable from measurements instead of guesses.
        SmartDashboard.putBoolean("Intake/Arm Stalled", self.is_stalled)
        SmartDashboard.putNumber("Intake/Arm Current (A)", self.arm_stator_current)
        SmartDashboard.putNumber("Intake/Arm Velocity (rps)", self.arm_velocity)
        SmartDashboard.putNumber("Intake/Arm Position (rot)", self.arm_position)

        # NetworkTables has no way to say "no value", so an idle arm reports -1 instead of a
        # position it was never commanded to.
        SmartDashboard.putNumber(
            "Intake/Commanded Position (rot)",
            self.commanded_position if self.commanded_position is not None else -1.0,
        )

    def set_intake_speed(self, intake_wheel_volts):
        """
        Apply the requested intake-wheel voltage.

        :param intake_wheel_volts: Voltage to apply to the intake wheel motor.
        :type intake_wheel_volts: float
        """
        self.intake_wheel.set_control(self.wheel_voltage_request.with_output(intake_wheel_volts))

    def set_setpoint(self, position):
        """
        Command the intake arm to the requested closed-loop position.

        :param position: Desired intake arm position in mechanism rotations.
        :type position: float
        """
        self.commanded_position = position

        # A new move starts clean, so clear any stall left over from the previous move before
        # commanding the arm again.
        self.is_stalled = False
        self.stall_timer.stop()
        self.stall_timer.reset()

        self.intake_arm.set_control(self.position_voltage_request.with_position(position))

    def detect_arm_movement_completion(self):
        """
        Report whether the arm has finished its move.

        :returns: True once the arm reaches the commanded position or a stall is recorded.
        :rtype: bool
        """
        if self.commanded_position is None:
            return False
        return (
            self.intake_arm.get_position().is_near(
                self.commanded_position, self._ARM_POSITION_TOLERANCE_ROT
            )
            or self.is_stalled
        )

    def set_arm_voltage(self, voltage):
        """
        Drive the intake arm with an open-loop voltage.

        :param voltage: Voltage to apply to the arm motor.
        :type voltage: float
        """
        self.intake_arm.set_control(self.arm_voltage_request.with_output(voltage))

    def arm_down(self):
        """Move the intake arm down to the floor-pickup position."""
        self.set_setpoint(self.intake_position)

    def arm_up(self):
        """Move the intake arm up to the stowed position."""
        self.set_setpoint(self.stowed_position)

    def arm_down_intermediate(self):
        """Move the intake arm to the intermediate position used while shooting."""
        self.set_setpoint(self.shooting_position)

    def get_stall_detection(self):
        """
        Watch the moving arm and cut power if it jams or stalls.

        Two cases stop the arm early. The first is a hard obstruction inside the pinch zone,
        meaning high current somewhere the arm should not be pushing against anything. The
        second is a general stall, where the arm pulls current but barely moves for long enough
        to rule out normal acceleration.

        :returns: True once a stall or obstruction has been detected, otherwise False.
        :rtype: bool
        """
        if self.commanded_position is None:
            return False

        # A stall latches until the next setpoint clears it, so a jammed arm stays stopped
        # instead of re-driving once the current briefly drops.
        if self.is_stalled:
            return True

        # The arm pinches against the frame across most of its travel, except near the shooting
        # position where it is expected to load up. High current inside that zone means something
        # is jammed, so stop before the motor overheats.
        in_obstruction_pathway = (
            self.arm_movement_pathway_low < self.arm_position < self.arm_movement_pathway_high
        )
        near_shooting_position = (
            abs(self.arm_position - self.shooting_position) <= self.obstruction_dead_band
        )
        if in_obstruction_pathway and not near_shooting_position:
            if self.arm_stator_current > self.obstruction_current_threshold:
                self.set_arm_voltage(0)
                self.is_stalled = True
                return self.is_stalled

        # General stall: still commanding a move, drawing current, but hardly moving.
        is_commanding_motion = (
            abs(self.commanded_position - self.arm_position) > self._ARM_POSITION_TOLERANCE_ROT
        )
        stall_condition_met = (
            is_commanding_motion
            and self.arm_stator_current > self.stall_current_threshold
            and self.arm_velocity < self.stall_velocity_threshold
        )

        # The timer only runs while the conditions hold, so a brief current spike during normal
        # acceleration resets it instead of counting toward a stall.
        if stall_condition_met:
            self.stall_timer.start()
        else:
            self.stall_timer.stop()
            self.stall_timer.reset()

        self.is_stalled = (
            self.stall_timer.hasElapsed(self.stall_time_threshold) and stall_condition_met
        )

        if self.is_stalled:
            self.set_arm_voltage(0)

        return self.is_stalled
