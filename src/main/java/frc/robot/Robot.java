// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.swervedrive.SwerveConstants;
import frc.robot.swervedrive.SwerveSubsystem;
import frc.robot.util.OdometryI;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  
  private final SwerveSubsystem swerveDrive = SwerveSubsystem.getInstance();
  private final SwerveDriveOdometry odometry = OdometryI.setInstance(
        SwerveConstants.KINEMATICS,
        swerveDrive.getRotation2d(),
        swerveDrive.getModulePositions()
  );
  
  private final Field2d field = new Field2d();

  public Robot() {
    SmartDashboard.putData(field);

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    odometry.update(swerveDrive.getRotation2d(), swerveDrive.getModulePositions());

    field.setRobotPose(odometry.getPoseMeters());

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
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
