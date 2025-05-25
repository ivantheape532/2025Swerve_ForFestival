package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ImprovedCommandXboxController;
import frc.robot.subsystems.SwerveDriveTrain;

public class RobotContainer {
    public static SwerveDriveTrain m_drivetrain = SwerveDriveTrain.getInstance();
    public static ImprovedCommandXboxController m_driverController = new ImprovedCommandXboxController(0);
    
    public RobotContainer() {
        configureBindings();
                RobotContainer.m_drivetrain.setDefaultCommand(new DriveCommand());
            }

    private void configureBindings() {
        // Add any additional button bindings here
    }
}