package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        // Physical constants
        public static final double TRACK_WIDTH = 0.5; // meters
        public static final double WHEEL_BASE = 0.5; // meters
        public static final double WHEEL_DIAMETER = 0.1; // meters
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
        
        // Swerve module constants
        public static final double DRIVE_GEAR_RATIO = 29./15.*60./15.; // SDS Mk4i L2 ratio
        public static final double ANGLE_GEAR_RATIO = 56./6.*60./10.;
        public static final double DRIVE_METERS_PER_TICK = WHEEL_CIRCUMFERENCE / (DRIVE_GEAR_RATIO * 2048);
        public static final double ANGLE_DEGREES_PER_TICK = 360.0 / (ANGLE_GEAR_RATIO * 2048);
        
        // PID constants for angle motors
        public static final double ANGLE_KP = 0.01;
        public static final double ANGLE_KI = 0.0;
        public static final double ANGLE_KD = 0.0;
        
        // Drive motor PID constants
        public static final double DRIVE_KP = 0.1;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0;
        
        // Max speeds
        public static final double MAX_SPEED = 3.0; // meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // radians per second
        
        // Module IDs
        public static final int FRONT_LEFT_DRIVE = 3;
        public static final int FRONT_LEFT_ANGLE = 4;
        public static final int FRONT_RIGHT_DRIVE = 7;
        public static final int FRONT_RIGHT_ANGLE = 8;
        public static final int BACK_LEFT_DRIVE = 5;
        public static final int BACK_LEFT_ANGLE = 6;
        public static final int BACK_RIGHT_DRIVE = 1;
        public static final int BACK_RIGHT_ANGLE = 2;
        
        // Analog encoder ports (if using analog encoders with SPX)
        public static final int FRONT_LEFT_ENCODER = 0;
        public static final int FRONT_RIGHT_ENCODER = 1;
        public static final int BACK_LEFT_ENCODER = 2;
        public static final int BACK_RIGHT_ENCODER = 3;
        
        // Angle offsets for each module (in degrees)
        public static final double FRONT_LEFT_OFFSET = 0.0;
        public static final double FRONT_RIGHT_OFFSET = 0.0;
        public static final double BACK_LEFT_OFFSET = 0.0;
        public static final double BACK_RIGHT_OFFSET = 0.0;
    }
    
    public static final class OIConstants {
        // public static final int port = 0; // 驱动控制器端口
        public static final double DEADBAND = 0.1; //死区
    }
}