// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystemMax extends SubsystemBase {
  // The motors on the left side of the drive.
  CANSparkMax m_leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port,MotorType.kBrushed);
  CANSparkMax m_leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port,MotorType.kBrushed);
  
  
  
  // The motors on the right side of the drive.
  CANSparkMax m_rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port,MotorType.kBrushed);
  CANSparkMax m_rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port,MotorType.kBrushed);
  MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
  MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
  
  RelativeEncoder m_leftEncoder = m_leftMotor1.getEncoder(Type.kQuadrature, 8128);
  RelativeEncoder m_rightEncoder = m_rightMotor1.getEncoder(Type.kQuadrature, 8128);
  SparkMaxPIDController m_rightPID = m_rightMotor1.getPIDController();
  public SparkMaxPIDController m_leftPID = m_leftMotor1.getPIDController();
  private double m_leftEncoderOffset = 0;
  private double m_rightEncoderOffset = 0;
  private NetworkTableEntry tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
  
  
  /*  Trying using follow mode, so we just control one motor and the other does the same thing.
  private final MotorControllerGroup m_leftMotors =
  new MotorControllerGroup(m_leftMotor1,m_leftMotor2);
  private final MotorControllerGroup m_rightMotors =
  new MotorControllerGroup(m_rightMotor1,m_rightMotor2);   
  */
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  
  // The gyro sensor
  private final Gyro m_gyro = new ADXRS450_Gyro();
  
  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;
  
  public double kp = DriveConstants.kp;
  public double ki = DriveConstants.ki;
  public double kd = DriveConstants.kd;
  private double m_autoTargetAngle;
  
  
  /** Creates a new DriveSubsystem. */
  
  public DriveSubsystemMax() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward.
    m_leftMotor1.setInverted(false);
    m_leftMotor2.setInverted(false);
    m_rightMotor1.setInverted(true);
    m_rightMotor2.setInverted(true);

    
    
    
    
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistancePerRevolution);
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    resetEncoders();
    m_gyro.calibrate();
    m_gyro.reset();
    
    
  }
  
  public void resetGyro() {
    m_gyro.reset();
  }
  
  /*  tankDrive for double joystick control where the 
  *  trigger speeds it up and there is a button to slow it down
  */
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
    double leftMotorPower = unDeadband(leftSpeed*speedFactor*1.1, 0.05, 0.1);
    double rightMotorPower = unDeadband(rightSpeed * speedFactor, 0.05, 0.1);
    tankDrive(leftMotorPower, rightMotorPower);
  }
  
  private void setLeftMotors(double d) {
    m_leftMotors.set(d);
  }
  
  private void setRightMotors(double d) {
    m_rightMotors.set(d);
  }
  
  
  private double unDeadband(double input, double deadband, double undeadband) {
    double speed = MathUtil.applyDeadband(input,deadband);
    if (speed == 0) { return 0;}
    return ((Math.abs(input) + undeadband) * (Math.abs(input)/input));
  }
  
  
  private void tankDrive(double leftSpeed, double rightSpeed) {
    //tankDrive(leftSpeed,rightSpeed,true);
    m_drive.tankDrive(leftSpeed, rightSpeed,true);
  }
  
  
  
  @Override
  public void periodic() {
    super.periodic();
    // Update the odometry in the periodic block
    m_odometry.update(
    m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
  }
  
  /**
  * Returns the currently-estimated pose of the robot.
  *
  * @return The pose.
  */
  public Pose2d getPose() {
    System.out.println( new Throwable().getStackTrace()[0].getMethodName());

    return m_odometry.getPoseMeters();
  }
  
  /**
  * Returns the current wheel speeds of the robot.
  *
  * @return The current wheel speeds.
  */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  public double getLeftWheelSpeed() {
    return m_leftEncoder.getVelocity();
  }
  
  public double getRightWheelSpeed() {
    return m_rightEncoder.getVelocity();
  }
  
  /**
  * Resets the odometry to the specified pose.
  *
  * @param pose The pose to which to set the odometry.
  */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_gyro.reset();
    if (null == pose) {
      throw new NullPointerException("pose cant be null");
    }
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }
  
  /**
  * Drives the robot using arcade controls.
  *
  * @param fwd the commanded forward movement
  * @param rot the commanded rotation
  */
  public void arcadeDrive(double fwd, double rot) {
    System.out.println("Rot:"+rot + " H:" + getHeading());
    m_drive.arcadeDrive(fwd, rot);
  }
  
  public void moveStraight(double speed){
    //Add 10% to left because it's slower.
    tankDrive(speed*1.1,speed);
  }
  
  public void forward() {
    tankDrive(0.5, 0.5);
  }

  public void backward() {
    tankDrive(-0.5, -0.5);
  }
  
  public void stop() {
    tankDrive(0,0);
  }
  
  /**
  * Controls the left and right sides of the drive directly with voltages.
  *
  * @param leftVolts the commanded left output
  * @param rightVolts the commanded right output
  */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor1.setVoltage(leftVolts);
    m_rightMotor1.setVoltage(rightVolts);
    m_drive.feed();
  }

 public void setWheelSpeeds(double leftSpeed, double rightSpeed) {
   System.out.println("Wheel Speeds: " + leftSpeed + " " + rightSpeed);
   //input is in meters per second I think.  Need to convert to motor speed.
   if  (! ( (leftSpeed < 3) && (leftSpeed > -3) ) ) {
    System.out.println("insane leftSpeed specified:" + leftSpeed);

    leftSpeed = 0;
 }
  if  (! ( (rightSpeed < 3) && (rightSpeed > -3) ) ) {
    System.out.println("insane rightSpeed specified:" + rightSpeed);
    rightSpeed = 0;
  }

   m_leftMotors.set(leftSpeed/10);
   m_rightMotors.set(rightSpeed/10);
 }
  
  /** Resets internal offsets.  The actual reset encoders function is asyncronous 
   * so you can get back the old counts for a while until the reset is sent over the 
   * CAN bus. Instead we simply store some offsets that we subtract off every time 
   * we get the position */
  public void resetEncoders() {


    m_leftEncoderOffset = m_leftEncoder.getPosition();
    m_rightEncoderOffset = m_rightEncoder.getPosition();
  }
  
  /**
  * Gets the average distance of the two encoders.
  *
  * @return the average of the two encoder readings
  */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
  }
  
  /**
  * Gets the left drive encoder.
  *
  * @return the left drive encoder position
  */
  public double getLeftEncoderPosition() {
    return (m_leftEncoder.getPosition() - m_leftEncoderOffset);
  }
  
  /**
  * Gets the right drive encoder position.
  *
  * @return the right drive encoder
  */
  public double getRightEncoderPosition() {
    return (m_rightEncoder.getPosition() - m_rightEncoderOffset);
  }
  
  /**
  * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
  *
  * @param maxOutput the maximum output to which the drive will be constrained
  */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }
  
  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }
  
  /**
  * Returns the heading of the robot.
  *
  * @return the robot's heading in degrees, from -180 to 180
  */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  
  /**
  * Returns the turn rate of the robot.
  *
  * @return The turn rate of the robot, in degrees per second
  */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }


  public void setTurnState(TrapezoidProfile.State leftState, TrapezoidProfile.State rightState) {
    
    double leftSpeed, rightSpeed, turned;
    leftSpeed = -leftState.velocity;
    rightSpeed = rightState.velocity;
    turned = getLeftEncoderPosition() - getRightEncoderPosition();

    System.out.println ("State: " + leftState.velocity + "-" + rightState.velocity + " | " 
    + leftState.position + " " + leftSpeed + " " + rightSpeed + " " + turned);
    
    m_drive.tankDrive(leftSpeed, rightSpeed);
    //m_drive.tankDrive(0,0);
  }


  public void turnDrive(double rot) {
    rot = MathUtil.applyDeadband(rot,0.05);
    if (rot != 0) {
      rot = MathUtil.clamp(rot,-0.4, 0.4);
      rot = (Math.abs(rot) + 0.4) * (Math.abs(rot)/rot);
    }
    m_drive.arcadeDrive(0, rot);
    //System.out.println("Rot: " + rot + "H:" + getHeading());
  }

  public double getAngleToTarget() {
    System.out.println("Age" + tx.getInfo().last_change +" " + (long) (System.currentTimeMillis() /1000)/60 );
    return tx.getDouble(0);
  }

  public void setupAutoTargetAngle() {
    System.out.println("Setting up Auto Targetting.  TX: " + tx.getDouble(0));
    m_autoTargetAngle = getHeading() - tx.getDouble(0);
  }

  public double getAutoTargetAngle() {
    return m_autoTargetAngle;
  }
}
