package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SwerveModule;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer;
  // private Double[] m_ListArm=new Double[40];
  // private Double[] m_ListTy=new Double[40];

  private SwerveModule swerve_modules_[] = new SwerveModule[];//todo
  
  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer(); 

    
  }


  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
      swerve_modules_[0].testModule();//todo
  }

  @Override
  public void testExit() {}
}
