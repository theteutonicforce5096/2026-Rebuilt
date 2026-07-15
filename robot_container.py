import commands2

from subsystems.intake import Intake

class RobotContainer:
    """
    Construct the robot subsystems, controller bindings, and autonomous chooser.
    """

    def __init__(self):
        """
        Create the shared robot subsystems and operator interface wiring.
        """
       
        # Create controller
        self.controller = commands2.button.CommandXboxController(0)

        # Create intake subsystem
        self.intake = Intake(50)
        
        self.create_button_bindings()


    def create_commands_teleop(self):
        """
        Reset operator-facing state and safe subsystem outputs for teleop.
        """
        pass
    
    def create_button_bindings(self):
        """
        Bind controller buttons to robot actions and default commands.
        """
       
        self.controller.povDown().onTrue(
            commands2.SequentialCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_setpoint(0)
                ),
                commands2.RepeatCommand(
                    self.intake.runOnce(
                        lambda: self.intake.get_stall_detection()
                    )
                ).withTimeout(1),
                # self.controller.povDown().onTrue(
            )
        )
        
        self.controller.povUp().onTrue(
        #     self.intake.runOnce(
        #         lambda: self.intake.arm_up()
        #     )
        # )
            commands2.SequentialCommandGroup(
                self.intake.runOnce(
                    lambda: self.intake.set_setpoint(10)
                ),
                commands2.RepeatCommand(
                    self.intake.runOnce(
                        lambda: self.intake.get_stall_detection()
                    )
                ).withTimeout(1),
                # self.controller.povDown().onTrue(
            )
        )
    
        self.controller.y().onTrue(
            self.intake.runOnce(
                lambda: self.intake.spin_motor(0)
            )
        )