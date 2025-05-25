package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveTrain extends SubsystemBase {

    private static SwerveDriveTrain instance = null;

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    public SwerveDriveKinematics kinematics;
    public SwerveDriveOdometry odometry;

    public static SwerveDriveTrain getInstance() {
        if (instance == null) {
            instance = new SwerveDriveTrain();
        }
        return instance;
    }

    public SwerveDriveTrain() {
        frontLeft = new SwerveModule(
                Constants.DriveConstants.FRONT_LEFT_DRIVE,
                Constants.DriveConstants.FRONT_LEFT_ANGLE,
                Constants.DriveConstants.FRONT_LEFT_ENCODER,
                Constants.DriveConstants.FRONT_LEFT_OFFSET,
                "Front Left");

        frontRight = new SwerveModule(
                Constants.DriveConstants.FRONT_RIGHT_DRIVE,
                Constants.DriveConstants.FRONT_RIGHT_ANGLE,
                Constants.DriveConstants.FRONT_RIGHT_ENCODER,
                Constants.DriveConstants.FRONT_RIGHT_OFFSET,
                "Front Right");

        backLeft = new SwerveModule(
                Constants.DriveConstants.BACK_LEFT_DRIVE,
                Constants.DriveConstants.BACK_LEFT_ANGLE,
                Constants.DriveConstants.BACK_LEFT_ENCODER,
                Constants.DriveConstants.BACK_LEFT_OFFSET,
                "Back Left");

        backRight = new SwerveModule(
                Constants.DriveConstants.BACK_RIGHT_DRIVE,
                Constants.DriveConstants.BACK_RIGHT_ANGLE,
                Constants.DriveConstants.BACK_RIGHT_ENCODER,
                Constants.DriveConstants.BACK_RIGHT_OFFSET,
                "Back Right");

        // Create kinematics using module positions (in meters)
        kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(Constants.DriveConstants.WHEEL_BASE / 2, Constants.DriveConstants.TRACK_WIDTH / 2),
                // Front right
                new Translation2d(Constants.DriveConstants.WHEEL_BASE / 2, -Constants.DriveConstants.TRACK_WIDTH / 2),
                // Back left
                new Translation2d(-Constants.DriveConstants.WHEEL_BASE / 2, Constants.DriveConstants.TRACK_WIDTH / 2),
                // Back right
                new Translation2d(-Constants.DriveConstants.WHEEL_BASE / 2, -Constants.DriveConstants.TRACK_WIDTH / 2));

        // Initialize odometry at origin facing forward
        odometry = new SwerveDriveOdometry(
                kinematics,
                Rotation2d.fromDegrees(0),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });
    }

    @Override
    public void periodic() {
        // Update odometry
        odometry.update(
                Rotation2d.fromDegrees(0), // Replace with gyro angle if available
                new SwerveModulePosition[] { // TODO
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                });

        // Log data
        SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
        SmartDashboard.putNumber("Odometry Rotation", odometry.getPoseMeters().getRotation().getDegrees());
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        ChassisSpeeds speeds;

        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, odometry.getPoseMeters().getRotation());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        }

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);

        // Normalize wheel speeds
        SwerveDriveKinematics.desaturateWheelSpeeds(
                moduleStates, Constants.DriveConstants.MAX_SPEED);

        // Set each module state
        frontLeft.setDesiredState(moduleStates[0]);
        frontRight.setDesiredState(moduleStates[1]);
        backLeft.setDesiredState(moduleStates[2]);
        backRight.setDesiredState(moduleStates[3]);
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(
                Rotation2d.fromDegrees(0), // Replace with gyro angle if available
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                },
                pose);
    }
}