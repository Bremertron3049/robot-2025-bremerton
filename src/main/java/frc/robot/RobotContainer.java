// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.swervedrive.SwerveSubsystem;
import frc.robot.swervedrive.commands.SwerveController;
import frc.robot.util.OdometryI;
import edu.wpi.first.math.geometry.Pose2d;

public class RobotContainer {
  
  private final SwerveSubsystem SwerveDrive = SwerveSubsystem.getInstance();
  private final SwerveDriveOdometry SwerveOdometry = OdometryI.getInstance();


  private final XboxController driveController = new XboxController(0);
  
  public RobotContainer() {

    //Ties the SwerveController command to the SwerveSubsystem.
    SwerveDrive.setDefaultCommand(new SwerveController(
      SwerveDrive,
      SwerveOdometry,
      () -> driveController.getLeftY(),
      () -> driveController.getLeftX(),
      () -> driveController.getRightX(),
      () -> (true),
      () -> driveController.getRightBumperButtonPressed(),
      () -> Pose2d.kZero)
    );

    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
