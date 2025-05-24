package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SwerveDriveTrain;

public class DriveCommand extends Command {
    public static SwerveDriveTrain drivetrain;
    public static XboxController controller;
  
  // Slew rate limiters to make joystick inputs more gentle
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
    
    public DriveCommand(SwerveDriveTrain drivetrain, XboxController controller) {
        // this.drivetrain = drivetrain;
        // this.controller = controller;       
        addRequirements(drivetrain);
    }
    
    @Override
    public void execute() {
        // Get joystick inputs
        double xSpeed = -controller.getLeftY();
        double ySpeed = -controller.getLeftX();
        double rot = -controller.getRightX();
        
        // Apply deadband
        xSpeed = Math.abs(xSpeed) > Constants.OIConstants.DEADBAND ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > Constants.OIConstants.DEADBAND ? ySpeed : 0.0;
        rot = Math.abs(rot) > Constants.OIConstants.DEADBAND ? rot : 0.0;
        
        // Apply slew rate limiting
        xSpeed = xLimiter.calculate(xSpeed) * Constants.DriveConstants.MAX_SPEED;
        ySpeed = yLimiter.calculate(ySpeed) * Constants.DriveConstants.MAX_SPEED;
        rot = rotLimiter.calculate(rot) * Constants.DriveConstants.MAX_ANGULAR_SPEED;
        
        // Drive the robot
        drivetrain.drive(xSpeed, ySpeed, rot, true);
    }
    
    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
}