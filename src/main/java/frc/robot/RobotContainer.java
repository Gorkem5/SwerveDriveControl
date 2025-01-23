package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController controller;

    public RobotContainer() {
        controller = new CommandXboxController(0);
        swerveSubsystem = new SwerveSubsystem(controller);

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // Example of a button binding
    }

    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return null;
    }
}
