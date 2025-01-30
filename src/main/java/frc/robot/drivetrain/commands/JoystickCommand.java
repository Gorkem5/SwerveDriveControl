package frc.robot.drivetrain.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.drivetrain.SwerveSubsystem;

public class JoystickCommand extends Command {
    SwerveSubsystem swerve;
    CommandXboxController controller;
    Translation2d start;
    boolean deadzoneEnabled = true;

    public JoystickCommand(SwerveSubsystem swerve, CommandXboxController controller) {
        this.swerve = swerve;
        this.controller = controller;

        addRequirements(swerve);
    }

    private double ApplyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone) {
            return 0;
        } else {
            return value;
        }
    }

    @Override
    public void initialize() {
        start = swerve.getRobotPose().getTranslation();
    }

    @Override
    public void execute() {

        double xSpeed = -controller.getLeftY();
        double ySpeed = controller.getLeftX();
        double rot = controller.getRightX();

        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        swerve.drive(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond, false);
        
        if (deadzoneEnabled) {
            xSpeed = ApplyDeadzone(xSpeed, DriveConstants.JOYDeadzone_X);
            ySpeed = ApplyDeadzone(ySpeed, DriveConstants.JOYDeadzone_Y);
            rot = ApplyDeadzone(rot, DriveConstants.JOYDeadzone_Rot);
        }

        
    }

}