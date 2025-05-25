package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.DriveCommand;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonSRX angleMotor;
    private final AnalogInput angleEncoder;
    private final PIDController anglePIDController;
    private final String name;
    private final double angleOffset;
  
  private Slot0Configs m_drive_slot0Configs = new Slot0Configs();
    
    public SwerveModule(int driveMotorID, int angleMotorID, int encoderPort, double angleOffset, String name) {
        driveMotor = new TalonFX(driveMotorID);
        angleMotor = new TalonSRX(angleMotorID);
        angleEncoder = new AnalogInput(encoderPort);
        this.angleOffset = angleOffset;
        this.name = name;
        
        // Configure drive motor
        // driveMotor.configFactoryDefault(); //TODO
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        // driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 0); //TODO
        // driveMotor.config_kP(0, Constants.DriveConstants.DRIVE_KP);
        // driveMotor.config_kI(0, Constants.DriveConstants.DRIVE_KI);
        // driveMotor.config_kD(0, Constants.DriveConstants.DRIVE_KD);

        m_drive_slot0Configs.kP=DriveConstants.DRIVE_KP;
        m_drive_slot0Configs.kI=DriveConstants.DRIVE_KI;
        m_drive_slot0Configs.kD=DriveConstants.DRIVE_KD;

        driveMotor.getConfigurator().apply(m_drive_slot0Configs);
        
        // Configure angle motor
        angleMotor.configFactoryDefault();
        angleMotor.setNeutralMode(NeutralMode.Brake);
        
        // Configure PID controller for angle
        anglePIDController = new PIDController(
            Constants.DriveConstants.ANGLE_KP,
            Constants.DriveConstants.ANGLE_KI,
            Constants.DriveConstants.ANGLE_KD
        );
        anglePIDController.enableContinuousInput(-180, 180);
    }
    
    public double getDrivePosition() {
        return driveMotor.getRotorPosition().getValueAsDouble() * Constants.DriveConstants.DRIVE_METERS_PER_TICK;
    }
    
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble() * Constants.DriveConstants.DRIVE_METERS_PER_TICK * 10;
    }
    
    public Rotation2d getAngle() {
        double voltage = angleEncoder.getVoltage();
        double angle = (voltage / RobotController.getVoltage5V()) * 360.0 - angleOffset;
        return Rotation2d.fromDegrees(angle);
    }
    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }
    
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to avoid spinning further than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, getAngle());
        
        // Set drive motor speed
        double driveOutput = optimizedState.speedMetersPerSecond / Constants.DriveConstants.MAX_SPEED;
        driveMotor.set(driveOutput); //TODO:control percentOutput
        
        // Set angle motor position using PID
        double angleOutput = anglePIDController.calculate(getAngle().getDegrees(), optimizedState.angle.getDegrees());
        angleMotor.set(ControlMode.PercentOutput, angleOutput);
        
        // Log data
        SmartDashboard.putNumber(name + " Target Angle", optimizedState.angle.getDegrees());
        SmartDashboard.putNumber(name + " Current Angle", getAngle().getDegrees());
        SmartDashboard.putNumber(name + " Drive Output", driveOutput);
    }
    
    public void stop() {
        driveMotor.set(0);
        angleMotor.set(ControlMode.PercentOutput,0.); //TODO:percentOutput
    }
}