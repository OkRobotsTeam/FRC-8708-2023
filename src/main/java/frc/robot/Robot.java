// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  // This function is run when the robot is first started up

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  // This function is called every 20 ms, no matter the mode.
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.autonomousInit();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // if a valid command was recieved, schedule it
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // This function is called periodically during autonomous.
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Cancels the autonomous task when teleop starts
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
  }

  // This function is called periodically during teleop
  @Override
  public void teleopPeriodic() {
  }
}