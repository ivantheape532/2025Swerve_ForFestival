// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Library.team95.BetterSwerveKinematics;
import frc.robot.Subsystems.ImprovedCommandXboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    // public static final class SemiAutoConstants{
    //     public static final double kSemiAutoVelocityP=2.6;//TODO
    //     public static final double kSemiAutoVelocityI=0.00;//TODO
    //     public static final double kSemiAutoVelocityD=0.0;//TODO
    //     public static final Constraints kSemiAutoVelocityConstrants =new Constraints(1,2);//TODO
    //       public static final double SemiAutoVelocitySlewRate=100;
    //     public static final double SemiAutoVelocityMax=1.5;
    //     public static final double kSemiAutoOmegaP=6;//TODO
    //     public static final double kSemiAutoOmegaI=0.0;//TODO
    //     public static final double kSemiAutoOmegaD=0.0;//TODO
    //     public static final Constraints kSemiAutoOmegaConstrants =new Constraints(2,100);//TODO
    //     public static final double SemiAutoOmegaSlewRate=100;
    //     public static final double SemiAutoOmegaMax=3;
    // }

    public static final class GlobalConstants {
        public static final double kLoopTime = 0.020;
        public static final float INF = (float)Math.pow(10, 5); // This represents the Infinite
    }
    public static final class DriveConstants {
        public static final double kMaxAcceleration = 3.0;
        public static final double kMaxAngularSpeed = Math.PI*1.5; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly
        public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly

        public static final double kInnerDeadband = 0.1; //TODO:0.1 // This value should exceed the maximum value the analog stick may
                                                          // read when not in use (Eliminates "Stick Drift")
        public static final double kOuterDeadband = 0.98; //TODO:0.98 // This value should be lower than the analog stick X or Y reading
                                                          // when aimed at a 45deg angle (Such that X and Y are are
                                                          // maximized simultaneously)
        public static final double kTranslationSlew = 10.0; //TODO:20.0
        public static final double kRotationSlew = 5;  //TODO:10

        public static final double kPositionTolerance = 0.05;    //TODO
        public static final double kRotationTolerance = 0.0;    //TODO
    }
    
    public static final class SwerveConstants {	
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
    
        public static final double driveGearRatio = 29./15.*60./15.;
        public static final double angleGearRatio = 56./6.*60./10.; 
        
        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.69552 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.8378 / 12); //TODO:2.8378 / 12
        public static final double driveKA = (0.44473 / 12);

        public static final double piviotKS = (1.322); //TODO:1.322 divide by 12 to convert from volts to percent output for CTRE
        public static final double piviotKV = (0.0052807);
        public static final double piviotKA = (0.44473 / 12);

        // Pigeon Port
        public static final int PigeonIMUPort = 0;

        public static double kDriveMotorMaxOutput = 1; //TODO:1 0.5
        public static double kPivotMotorMaxOutput = 1; //TODO:1 0.1
    
        public static double kDriveMotorNeutralDeadband = 0;
        public static double kPivotMotorNeutralDeadband = 1; //TODO:0
        
        public static double VelocityCorrectionFactor = 0.5;//TODO
        /**
         * The first version of PID parameters
         *                         Value
         * @param kDriveMotorkP    0.025
         * @param kDriveMotorkI    0.0016
         * @param kDriveMotorkD    2.5
         * @param kDriveMotorkF    0.06
         * @param kDriveMotorIZone 240
         * 
         * 车子在行驶过程中基本不抖动，底盘PID大部分情况下是正常的。
         * 
         * *不过到我这可就不一定了 ——zyf 2025.5.4
         * 
         */
        public static double kDriveMotorkP = 0.312; // 5e-2 0.05   0.025
        public static double kDriveMotorkI = 0; //5e-4 0.005  0.0016
        public static double kDriveMotorkD = 0.; //   5e-0 5 1.5  2.5
        public static double kDriveMotorkF = 0.054;//   0.045       0.06
        public static double kDriveMotorIZone = 0;// 90          240
        public static double kSensorVelocityMeasPeriod = 10;
    
        public static double kPivotMotorkP = 2;//3
        public static double kPivotMotorkI = 0;
        public static double kPivotMotorkD = 100.;//100
        public static double kPivotMotorF = 0;
        public static double kPivotMotorkIZone = 0;
        public static double motionCruiseVelocity = 1200;
        public static double motionAcceleration = 3500;    
    
        public static double kLoopSeconds = 0.0;
    
        public final static double kDriveMotorReductionRatio = 1.0 / (29./15*60/15); //29/15*60/15
        public final static double kPivotMotorReductionRatio = 1.0 / (56./6*60/10); //56/6*60/10
        public final static double kDriveEncoderReductionRatio = 1.0 / (29. / 15 * 60 / 15);
        public final static double kDriveEncoderReductionRatioTest = (29. / 15 * 60 / 15);
        public static double kPivotEncoderReductionRatio = -1.0 / 1.0;
    
        public static final double FALCON_TICS_PER_ROTATION = 2048.0;
        public static final double TALON_TICS_PER_ROTATION = 4096.0;
    
        public static double kDriveEncoderResolution = 2048.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad
        public static double kPivotEncoderResolution = 4096.0 / (Math.PI * 2);//the unit of "2"is rad 2_rad
    
        public static double kWidth  = 0.464178;//The unit is 0.342_m
        public static double kLength = 0.464178;//The unit is 0.342_m
    
        public static double kWheelDiameter = 0.093;//The unit is meter
    
        public static double kPeriod = 20;//The unit is 20_ms
    
        public static double kMaxSpeed = 5.;//TODO: 5. The unit is meters per second 每个轮子的可以达到的最大转速，单位是m/s
    
       
        public static final BetterSwerveKinematics swerveKinematics = new BetterSwerveKinematics(//define the position of each swervemodule by creating a coordinate system
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(kLength / 2.0, -kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, kWidth / 2.0),
            new edu.wpi.first.math.geometry.Translation2d(-kLength / 2.0, -kWidth / 2.0));
    
        public static final double wheelCircumference = kWheelDiameter * Math.PI;
    
        public static final double MAX_SPEED_METERSperSECOND = 21900/0.1/(driveGearRatio*2048)*wheelCircumference;
        public static final double METERSperROBOT_REVOLUTION =  2*Math.PI*(kWidth/2*1.414213);//Math.sqrt(2)
        public static final double MAX_SPEED_RADIANSperSECOND = MAX_SPEED_METERSperSECOND / METERSperROBOT_REVOLUTION
                * (2 * Math.PI);
        
        public static final int SWERVE_MOTOR_CURRENT_LIMIT = 15;
        public static final int SWERVE_DRIVE_MOTOR_CURRENT_LIMIT = 15;
        public static final int SWERVE_DRIVE_VOLTAGE_LIMIT = 12;
    }


}

