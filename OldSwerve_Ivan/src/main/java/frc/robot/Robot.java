package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//import com.ctre.phoenix6.swerve.SwerveModule;

import frc.robot.subsystems.SwerveModule;

public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private final SwerveModule testSwerveModule;
    testSwerveModule = new SwerveModule();// todo
    
    @Override
    public void robotInit() {
        robotContainer = new RobotContainer();
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
    public void teleopPeriodic() {
        // testSwerveModule.setDesiredState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)));
    }
    
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
    
    @Override
    public void testPeriodic() {
        testSwerveModule.setDesiredState(new SwerveModuleState(1.0, Rotation2d.fromDegrees(0)));
    }
}