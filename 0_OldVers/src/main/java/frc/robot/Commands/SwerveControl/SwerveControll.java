
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.SwerveControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Library.team1706.MathUtils;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveControll extends Command {
  /** Creates a new SwerveControll. */
  //double storedYaw;
  private boolean fieldOrient = true; //true
  //That means the joystick will reach the max range in 1/3 second
  //The may let the robot move smoothly.
  private final SlewRateLimiter m_slewX = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewY = new SlewRateLimiter(DriveConstants.kTranslationSlew);
  private final SlewRateLimiter m_slewRot = new SlewRateLimiter(DriveConstants.kRotationSlew);
  
  // private double rotation;
  // private Translation2d translation;
  //That means reaching the max speed will take 1/3 seconds.The max Liner Speed is 4m/s. Omega is 
  // private SlewRateLimiter xSpeedLimiter = new SlewRateLimiter(Constants.linarslewrate * Constants.kMaxSpeed);
  // private SlewRateLimiter ySpeedLimiter = new SlewRateLimiter(Constants.linarslewrate * Constants.kMaxSpeed);
  // private SlewRateLimiter omegaSpeedLimiter = new SlewRateLimiter(Constants.omegaslewrate * Constants.kMaxOmega);

  public SwerveControll() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // storedYaw = RobotContainer.m_swerve.GetYaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
public void execute() {
    // 获取左摇杆的输入，用于控制移动
    double translationX = -inputTransform(RobotContainer.m_driverController.getLeftY()); // 前后移动
    double translationY = -inputTransform(RobotContainer.m_driverController.getLeftX()); // 左右移动

    // 获取右摇杆的输入，用于控制旋转
    double rotationNew = -inputTransform(RobotContainer.m_driverController.getRightX()); // 旋转

    // 使用 SlewRateLimiter 平滑控制输入
    Translation2d translation = new Translation2d(
        m_slewX.calculate(translationX) * SwerveConstants.kMaxSpeed,
        m_slewY.calculate(translationY) * SwerveConstants.kMaxSpeed
    );

    // 调用驱动方法，传入平滑后的移动和旋转值
    RobotContainer.m_swerve.Drive(
        translation,
        m_slewRot.calculate(rotationNew) * DriveConstants.kMaxAngularSpeed,
        fieldOrient, // 是否使用场地方向
        RobotContainer.m_swerve.isOpenLoop // 是否为开环控制
    );
}
  // @Override
  // public void execute() {
  //   double translationX = -inputTransform(RobotContainer.m_driverController.getLeftY());
  //   double translationY = -inputTransform(RobotContainer.m_driverController.getLeftX());
  //   double rotationNew = -inputTransform(RobotContainer.m_driverController.getRightX());
    
  //   Translation2d translation = new Translation2d(m_slewX.calculate(
  //     translationX) * SwerveConstants.kMaxSpeed,
  //       m_slewY.calculate(translationY) * SwerveConstants.kMaxSpeed);
  //   if(DriverStation.getAlliance().get()==Alliance.Red)
  //   {
  //     translation=translation.unaryMinus();
  //   }
  //   RobotContainer.m_swerve.Drive(
  //       translation,
  //       m_slewRot.calculate(rotationNew) * DriveConstants.kMaxAngularSpeed,
  //       fieldOrient,
  //       RobotContainer.m_swerve.isOpenLoop);

  //   /**
  //    * The methods below are a try based on 1678's swerve control
  //    */
      
  //   /*double llastx = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftY());
  //   double llasty = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getLeftX());
  //   double llastz = RobotContainer.m_swerve.deadband(RobotContainer.m_driverController.getRightX());

  //   double yaw = RobotContainer.m_swerve.GetYaw();
  //   double yawCorrection = 0;

  //   //storedYaw = yaw; //This may be a bug!*/

  //   /*if(llastz != 0){
  //     storedYaw = yaw;
  //   }
  //   else{
  //     if(Math.abs(llastx) > 0|| Math.abs(llasty) > 0){
  //       yawCorrection = RobotContainer.m_swerve.calcYawStraight(storedYaw, yaw, 0.01, 0);
  //     }
  //   }*/

  //   /*RobotContainer.m_swerve.Drive(
  //     - RobotContainer.m_swerve.deadband(llastx) * Constants.kMaxSpeed, 
  //     - RobotContainer.m_swerve.deadband(llasty) * Constants.kMaxSpeed, 
  //     -  (llastz + yawCorrection) * Constants.kMaxOmega, //Add Drive straight mode
  //     true);*/
    
  //   //SmartDashboard.putNumber("yaw", yaw);
  //   //SmartDashboard.putNumber("storedyaw", storedYaw);
  //   //SmartDashboard.putNumber("llastz", llastz);
  //   /*SmartDashboard.putNumber("yawcorrection", yawCorrection);
  //   SmartDashboard.putBoolean("WhetherStroeYaw", RobotContainer.m_swerve.whetherstoreyaw);

  //   //Try to optimize the input
  //   double yAxis;
  //   double xAxis;
  //   double rAxis;

  //   Translation2d tAxes; // translational axis*/

  //   /* Inversions */
  //   /*yAxis = -RobotContainer.m_driverController.getLeftY();
  //   xAxis = -RobotContainer.m_driverController.getLeftX();
  //   rAxis = RobotContainer.m_driverController.getRightX();*/

  //   /* Deadbands */
  //   /*tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
  //   rAxis = applyRotationalDeadband(rAxis);

  //   translation = new Translation2d(tAxes.getX(), tAxes.getY()).times(Constants.kMaxSpeed);
  //   rotation = rAxis * Constants.kMaxOmega;*/

  //   /*rotation = (rAxis + yawCorrection) * Constants.kMaxOmega;*/

  //   /*RobotContainer.m_swerve.Drive(
  //       translation,
  //       -(llastz + yawCorrection) * Constants.kMaxOmega,
  //       true,
  //       RobotContainer.m_swerve.isOpenLoop);*/

  //   /*if(RobotContainer.m_swerve.whetherstoreyaw || llastz != 0){
  //     storedYaw = yaw;
  //   }else{
  //     if(Math.abs(llastx) > 0|| Math.abs(llasty) > 0){
  //       yawCorrection = RobotContainer.m_swerve.calcYawStraight(storedYaw, yaw, 0.01, 0);
  //     }
  //   }*/

  //   /*if(RobotContainer.m_swerve.whetherstoreyaw || rotation != 0){
  //     storedYaw = yaw;
  //   }else{
  //     if(Math.abs(tAxes.getX()) > 0|| Math.abs(tAxes.getY()) > 0){
  //       yawCorrection = RobotContainer.m_swerve.calcYawStraight(storedYaw, yaw, 0.01, 0);
  //     }
  //   }*/
  
  //   //SmartDashboard.putNumber("X Controller Input", translation.getX());
  //   //SmartDashboard.putNumber("Y Controller Input", translation.getY());
  //   //SmartDashboard.putNumber("Rot Controller Input", rotation);
  //   //SmartDashboard.putNumber("storedYaw", storedYaw);
  //   //SmartDashboard.putNumber("origin_yaw", yaw);

  // }

  // // Called once the command ends or is interrupted.
  // @Override
  // public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }

  /**
   * when this fucntion of the command is called the current fieldOrient boolean
   * is flipped. This
   * is fed into the drive command for the swerve drivetrain so the driver can
   * decide to drive in
   * a robot oreinted when they please (not recommended in most instances)
   */
  public void changeFieldOrient() {
    if (fieldOrient) {
      fieldOrient = false;
    } else {
      fieldOrient = true;
    }
  }

  /**
   * This function takes the user input from the controller analog sticks, applys
   * a deadband and then quadratically
   * transforms the input so that it is easier for the user to drive, this is
   * especially important on high torque motors
   * such as the NEOs or Falcons as it makes it more intuitive and easier to make
   * small corrections
   * 
   * @param input is the input value from the controller axis, should be a value
   *              between -1.0 and 1.0
   * @return the transformed input value
   */
  private double inputTransform(double input) {
    return MathUtils.signedSquare(MathUtils.applyDeadband(input));
  }

    /*private double applyRotationalDeadband(double input){
    double deadband = Constants.stickDeadband;
    if (Math.abs(input) < deadband) {
        return 0.0;
    } else {
        return (input - (Math.signum(input) * deadband)) / (1 - deadband);
    }
  }

  private Translation2d applyTranslationalDeadband(Translation2d input) {
    double deadband = Constants.stickDeadband;
    if (Math.abs(input.getNorm()) < deadband) {
        return new Translation2d();
    } else {
        Rotation2d deadband_direction = new Rotation2d(input.getX(), input.getY());
        Translation2d deadband_vector = new Translation2d(deadband, deadband_direction);

        double scaled_x = input.getX() - (deadband_vector.getX()) / (1 - deadband_vector.getX());
        double scaled_y = input.getY() - (deadband_vector.getY()) / (1 - deadband_vector.getY());
        return new Translation2d(scaled_x, scaled_y);
    }

  }

  public double[] getAxes() {
    double yAxis = -RobotContainer.m_driverController.getLeftY();
    double xAxis = -RobotContainer.m_driverController.getLeftX();
    double rAxis = -RobotContainer.m_driverController.getRightX();

    Translation2d tAxes;*/
    
    /* Deadbands */
    /*tAxes = applyTranslationalDeadband(new Translation2d(yAxis, xAxis));
    rAxis = applyRotationalDeadband(rAxis);

    double[] axes = {tAxes.getX(), tAxes.getY(), rAxis};

    return axes;
  }*/
}