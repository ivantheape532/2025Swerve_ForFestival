// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Commands.SwerveControl.SwerveControll;
import frc.robot.Subsystems.*;
import frc.robot.Subsystems.ImprovedCommandXboxController;

public class RobotContainer {
  public static SwerveDriveTrain m_swerve = SwerveDriveTrain.getInstance();
  public static ImprovedCommandXboxController m_driverController= new ImprovedCommandXboxController(0);
  public RobotContainer() {
    configureBindings();
    
   RobotContainer.m_swerve.setDefaultCommand(new SwerveControll()); //TODO
   
  }

  private void configureBindings() {
  }

}
