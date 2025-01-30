package frc.robot;

public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class DriveConstants {
        public static final double JOYDeadzone_X = 0.1;
        public static final double JOYDeadzone_Y = 0.1;
        public static final double JOYDeadzone_Rot = 0.1;

        public static final double MaxDriveSpeed = 3.0; // Example value in meters per second
        public static final double MaxRotSpeed = 1.0; // Example value in radians per second

        public static final double MaxDriveAccel = 2.0; // Example value in meters per second squared
        public static final double MaxRotAccel = 1.0; // Example value in radians per second squared
    }

    public static final class SwerveConstants {
        public static final int AngleCANID_FL = 1;
        public static final int DriveCANID_FL = 2;
        public static final int AngleCANID_FR = 3;
        public static final int DriveCANID_FR = 4;
        public static final int AngleCANID_BL = 5;
        public static final int DriveCANID_BL = 6;
        public static final int AngleCANID_BR = 7;
        public static final int DriveCANID_BR = 8;

        public static final double ModuleOffsetM_X = 0.381; // Example value in meters
        public static final double ModuleOffsetM_Y = 0.381; // Example value in meters
    }
}