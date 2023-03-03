// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystemOld extends SubsystemBase {
  // The motors on the left side of the drive.
  private final MotorControllerGroup m_leftMotors =
      new MotorControllerGroup(
          new Spark(DriveConstants.kLeftMotor1Port),
          new Spark(DriveConstants.kLeftMotor2Port));

  // The motors on the right side of the drive.
  private final MotorControllerGroup m_rightMotors =
      new MotorControllerGroup(
        new Spark(DriveConstants.kRightMotor1Port),
        new Spark(DriveConstants.kRightMotor2Port));
  
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The left-side drive encoder


  /** Creates a new DriveSubsystem. */
  public DriveSubsystemOld() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftMotors.setInverted(true);
    

  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  /*public void tankDrive(double leftSpeed, double rightSpeed, double trigger) {
    double speedFactor = (0.3*trigger)+0.7;
    SmartDashboard.putNumber("speedFactor", speedFactor);
    m_drive.tankDrive(leftSpeed*speedFactor, rightSpeed*speedFactor);
  }*/
  public void tankDrive(double leftSpeed, double rightSpeed, Boolean trigger, Boolean slowButton) {
    double speedFactor = 0.7;
    if (trigger){
      if (slowButton){
        speedFactor = 0.7;
      } else {
        speedFactor = 1;
      }
    } else if (slowButton){
      speedFactor = 0.5;
    }
    SmartDashboard.putNumber("speedFactor", speedFactor);
    m_drive.tankDrive(leftSpeed*speedFactor, rightSpeed*speedFactor);
  }
  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public void setDriveStates(State leftState, State rightState) {
    double leftSpeed = leftState.velocity/5700;
    double rightSpeed = rightState.velocity/5700;
    m_leftMotors.set(leftSpeed);
    m_rightMotors.set(rightSpeed);
  }

  public void resetEncoders() {
    //I don't have any
  }

public void resetOdometry(Pose2d initialPose) {
  //I cant
}

public void  tankDriveVolts(int i, int j) {
    m_leftMotors.set(i);
    m_rightMotors.set(j);
}
}
