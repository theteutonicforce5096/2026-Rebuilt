from commands2 import PrintCommand, Subsystem, SequentialCommandGroup
from wpilib import SmartDashboard
import wpilib
from wpimath.controller import ProfiledPIDController, PIDController
from wpimath.trajectory import TrapezoidProfile
from wpilib import Timer
import rev

class Intake(Subsystem):
    """
    Class for controlling intake.
    """

    def __init__(self, CAN_ID):
        
        Subsystem.__init__(self) 
        
        # Create motors
        self.intake_arm = rev.SparkMax(CAN_ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.intake_arm_encoder = self.intake_arm.getEncoder()

        # Apply motor configs
        intake_arm_configs = (
            rev.SparkMaxConfig()
            .voltageCompensation(12.0)
            .setIdleMode(rev.SparkBaseConfig.IdleMode.kBrake)
            .smartCurrentLimit(40)
            .inverted(False)
        )

        self.intake_arm.configure(
            intake_arm_configs,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kNoPersistParameters
        )

        self.pid_controller = ProfiledPIDController(1, 0, 0, TrapezoidProfile.Constraints(35, 16))

        self.position = 0
        self.intake_arm_encoder.setPosition(0.0)

        # Arm Positions because apparently we need those
        self.intake_position = 0
        self.stowed_position = 10
        self.shooting_position = 5 

        self.set_position = 0

        # Stall Detection 
        self.stall_current_threshold = 30
        self.stall_velocity_threshold = .25
        self.stall_time_threshold = .25
        self.stall_timer = Timer()
        self.is_stalled = False
        self.last_command_output = 0.0
        self.last_time = 0.0

    def periodic(self):
        """
        Current and velocity for stall detection
        """
        self.now = wpilib.getTime()
        self.dt = self.now - self.last_time
        self.last_time = self.now
        self.current = self.intake_arm.getOutputCurrent()
        self.velocity = self.intake_arm_encoder.getVelocity()
        self.intake_arm_now = self.intake_arm_encoder.getPosition()

        print(self.set_position)
        print(self.intake_arm_encoder.getPosition())

    def get_stall_detection(self):
        is_commanding_motion = abs(self.set_position - self.intake_arm_now) > .05 # Should be False

        stall_condition_met = (
            is_commanding_motion
            and self.current > self.stall_current_threshold
            and abs(self.velocity) < self.stall_velocity_threshold
        )
        
        if self.set_position is None:
            return

        if stall_condition_met:
            self.stall_timer.start()

        else:
            self.stall_timer.stop()
            self.stall_timer.reset()

        self.is_stalled = self.stall_timer.hasElapsed(self.stall_current_threshold) and stall_condition_met

        if self.is_stalled:
            print("help me")
            
            self.set_setpoint(self.intake_arm_now) # set the setpoint to the current position to the position that it's at RIGHT NOW
            self.run_pid() #TODO test

        return self.is_stalled

    # Create PID control requests
    def spin_motor(self, percent):
        self.intake_arm.set(percent)

    def run_pid(self):
        current_position = self.intake_arm_encoder.getPosition()
        output = self.pid_controller.calculate(current_position, self.set_position)
        self.spin_motor(output)

    def set_setpoint(self, position):
        """
        Command the intake arm to the requested closed-loop position.

        :param position: Desired intake arm position in mechanism rotations.
        :type position: float
        """
        self.set_position = position

        # print(position)