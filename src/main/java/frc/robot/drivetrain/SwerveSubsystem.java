package frc.robot.drivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, odometry.getPoseMeters().getRotation());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }
        setChassisSpeed(chassisSpeeds);
    }

    public Pose2d getRobotPose() {
        return odometry.getPoseMeters();
    }

    SwerveModule frontLeftModule = new SwerveModule(0, 1);
    SwerveModule frontRightModule = new SwerveModule(2, 3);
    SwerveModule backLeftModule = new SwerveModule(4, 5);
    SwerveModule backRightModule = new SwerveModule(6, 7);

    // Define an array of swerve modules
    SwerveModule[] modules = {frontLeftModule, frontRightModule, backLeftModule, backRightModule};

    // Define a kinematics object
    double chassisWidth = Units.inchesToMeters(32);
    double chassisLength = Units.inchesToMeters(32);

    Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
    Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
    Translation2d backLeftLocation = new Translation2d(-chassisLength / 2, chassisWidth / 2);
    Translation2d backRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth / 2);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        frontLeftLocation,
        frontRightLocation,
        backLeftLocation,
        backRightLocation
    );

    private final Field2d odomDisplay = new Field2d();
    private final SwerveDriveOdometry odometry;
    private final CommandXboxController controller;

    public SwerveSubsystem(CommandXboxController io) {
        System.out.println("SwerveSubsystem Constructor");
        controller = io;
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), getModulePositions());
        SmartDashboard.putData("Field", odomDisplay);
    }

    public void setChassisSpeed(ChassisSpeeds desired) {
        // Get the desired states of the wheels
        SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desired);

        // Optimize the states of each module (speed/direction)
        newStates[0] = optimizeModuleState(newStates[0], frontLeftModule.getState().angle);
        newStates[1] = optimizeModuleState(newStates[1], frontRightModule.getState().angle);
        newStates[2] = optimizeModuleState(newStates[2], backLeftModule.getState().angle);
        newStates[3] = optimizeModuleState(newStates[3], backRightModule.getState().angle);

        // Set the states of each module (speed/direction)
        frontLeftModule.setState(newStates[0]);
        frontRightModule.setState(newStates[1]);
        backLeftModule.setState(newStates[2]);
        backRightModule.setState(newStates[3]);
    }

    // Optimize the swerve module state to minimize the rotation needed to achieve the desired angle
    public SwerveModuleState optimizeModuleState(SwerveModuleState state, Rotation2d currentAngle) {
        double targetAngle = state.angle.getDegrees();
        double currentAngleDegrees = currentAngle.getDegrees();
        double deltaAngle = targetAngle - currentAngleDegrees;

        // Ensure the delta angle is within the range [-180, 180]
        deltaAngle = (deltaAngle + 180) % 360 - 180;

        // If the delta angle is greater than 90 degrees, reverse the wheel direction
        if (Math.abs(deltaAngle) > 90) {
            deltaAngle -= Math.signum(deltaAngle) * 180;
            state = new SwerveModuleState(-state.speedMetersPerSecond, Rotation2d.fromDegrees(targetAngle + deltaAngle));
        } else {
            state = new SwerveModuleState(state.speedMetersPerSecond, Rotation2d.fromDegrees(targetAngle + deltaAngle));
        }

        return state;
    }

    public ChassisSpeeds getChassisSpeeds() {
        SwerveModuleState[] states = getModuleStates();
        return kinematics.toChassisSpeeds(states);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // Read the data from the controller
        ChassisSpeeds newDesiredSpeed = new ChassisSpeeds(
            // Pushing forward will ask the robot to go forward
            controller.getLeftY(),
            // Pushing left will ask the robot to go left
            controller.getLeftX(),
            // Pushing right will ask the robot to go right
            controller.getRightX()
        );

        setChassisSpeed(newDesiredSpeed);

        // FL, FR, BL, BR
        double loggingState[] = {
            frontLeftModule.getState().angle.getDegrees(),
            frontRightModule.getState().angle.getDegrees(),
            backLeftModule.getState().angle.getDegrees(),
            backRightModule.getState().angle.getDegrees()
        };

        // Log the state of each module (for debugging purposes)
        System.out.println("FL: " + loggingState[0] + " FR: " + loggingState[1] + " BL: " + loggingState[2] + " BR: " + loggingState[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = new SwerveModulePosition(modules[i].getDistanceMeters(), modules[i].getState().angle);
        }
        return positions;
    }
}