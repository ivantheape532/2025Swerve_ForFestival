package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveDriveTrain;

public class RobotContainer {
    public static SwerveDriveTrain m_drivetrain = SwerveDriveTrain.getInstance();
    public static XboxController m_driverController = new XboxController(Constants.OIConstants.DRIVER_CONTROLLER_PORT);
    
    public RobotContainer() {
        configureButtonBindings();
        RobotContainer.m_drivetrain.setDefaultCommand(new DriveCommand(m_drivetrain, m_driverController));
    }
    
    private void configureButtonBindings() {
        // Add any additional button bindings here
    }
}