package frc.robot.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.math.geometry.Rotation2d;

public class SwerveModule {
    // Define all the variables that will be needed
    private final Spark driveMotor;
    private final Spark steeringMotor;
    private final Encoder driveEncoder;
    private final Encoder steeringEncoder;
    private SwerveModuleState currentState;

    // PID Controllers
    private final PIDController drivePIDController;
    private final PIDController steeringPIDController;

    // Constructor
    public SwerveModule(int driveMotorPort, int steeringMotorPort) {
        System.out.println("SwerveModule Constructor");
        driveMotor = new Spark(driveMotorPort);
        steeringMotor = new Spark(steeringMotorPort);
        driveEncoder = new Encoder(driveMotorPort, driveMotorPort + 1); // Assuming encoder ports are consecutive
        steeringEncoder = new Encoder(steeringMotorPort, steeringMotorPort + 1); // Assuming encoder ports are consecutive
        currentState = new SwerveModuleState();

        // Initialize the PID controllers
        drivePIDController = new PIDController(1, 0, 0);
        steeringPIDController = new PIDController(1, 0, 0);
        steeringPIDController.enableContinuousInput(-180, 180);
    }

    // Method to get the distance in meters
    public double getDistanceMeters() {
        // Assuming the drive motor has an encoder that returns distance in meters
        return driveEncoder.getDistance();
    }

    // Method to get the current angle of the module
    public Rotation2d getAngle() {
        // Assuming the steering motor has an encoder that returns angle in degrees
        return Rotation2d.fromDegrees(steeringEncoder.getDistance());
    }

    public SwerveModuleState getState() {
        return currentState;
    }

    public void setState(SwerveModuleState newState) {
        double driveOutput = drivePIDController.calculate(driveEncoder.getRate(), newState.speedMetersPerSecond);
        double steeringOutput = steeringPIDController.calculate(steeringEncoder.getDistance(), newState.angle.getDegrees());

        driveMotor.set(driveOutput);
        steeringMotor.set(steeringOutput);

        currentState = newState;
    }
}