// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.AutoRetractHanger;
import frc.robot.commands.TurnToAngle;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    /*  For testing auto turning 
    SmartDashboard.putNumber("kp", Constants.DriveConstants.kp);
    SmartDashboard.putNumber("ki", Constants.DriveConstants.ki);
    SmartDashboard.putNumber("kd", Constants.DriveConstants.kd);
    SmartDashboard.putNumber("Angle",90);
    SmartDashboard.updateValues();
    */

    SmartDashboard.putNumber("Lights", 0);
    SmartDashboard.updateValues();
    m_robotContainer = new RobotContainer();
    m_robotContainer.setupJoystickControll();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.


    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Shooter Speed", m_robotContainer.m_shooter.m_shooterTargetSpeed);

    SmartDashboard.updateValues();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    System.out.println("Retracting hook");
    m_robotContainer.m_hook.retract();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.m_visionThread.getCenter();

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    //m_robotContainer.m_robotDrive.kp = SmartDashboard.getNumber("kp",0);
    //m_robotContainer.m_robotDrive.ki = SmartDashboard.getNumber("ki",0);
    //m_robotContainer.m_robotDrive.kd = SmartDashboard.getNumber("kd",0);
    //double angle = SmartDashboard.getNumber("Angle",90);
    //System.out.println("kp" + m_robotContainer.m_robotDrive.kp);
    //m_autonomousCommand = new TurnToAngle(angle,m_robotContainer.m_robotDrive);
    //m_robotContainer.setupDefaultStopped();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.m_robotDrive.resetEncoders();
    m_robotContainer.m_robotDrive.resetGyro();
    m_robotContainer.m_shooter.setTargetSpeed(ShooterConstants.kTwoBallRPM);
 
    //m_robotContainer.m_robotDrive.setDefaultCommand( new InstantCommand( () -> {}, m_robotContainer.m_robotDrive));
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    System.out.println("Starting Autonomous Command Now" + m_autonomousCommand);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.setupJoystickControll();
    m_robotContainer.m_shooter.stopFeeder();
    m_robotContainer.m_shooter.stopFlywheel();
    m_robotContainer.m_pickup.pickupUp();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_robotContainer.updateShooterSpeed();
    m_robotContainer.m_lights.setLights( SmartDashboard.getNumber("Lights",0));
    SmartDashboard.updateValues();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
