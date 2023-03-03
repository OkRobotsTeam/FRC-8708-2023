// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
* The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
* constants. This class should not be used for any other purpose. All constants should be declared
* globally (i.e. public static). Do not put anything functional in this class.
*
* <p>It is advised to statically import this class (or one of its inner classes) wherever the
* constants are needed, to reduce verbosity.
*/
public final class Constants {
  public static final int kTransferMotor1 = 6;
  public static final int kTransferMotor2 = 7;
  public static final int kPickupMotor = 5;

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 4;
    public static final int kLeftMotor2Port = 3;
    public static final int kRightMotor1Port = 1;
    public static final int kRightMotor2Port = 2;
    public static final boolean kRightEncoderReversed = false;
    public static final boolean kLeftEncoderReversed = true;
    
    public static final double kWheelDiameterInches = 6;
    public static final double kEncoderDistancePerRevolution = (kWheelDiameterInches * Math.PI);
    public static final double kMaxTurnRateDegreesPerS = 180;
    public static final double kMaxTurnAccDegreesPerSS = 180;
    

    
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kTrackwidthMeters = 0.558;  //22 inches = 0.558 meters
    public static final double kMetersPerRadian = kTrackwidthMeters / 2;
    public static final double kMetersPerDegree = kMetersPerRadian / 57.2958 ;

    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static double ks = 0.63;
    public static double kv = 3.26;
    public static double ka = 0.71;
    public static double kp = 0.018;
    public static double ki = 0;
    public static double kd = 0.0019;

    public static double kPDriveVel = 0.1;
    public static double kTurnToleranceDeg = 3;
    public static double kTurnRateToleranceDegPerS = 3;
    public static double turnTimeout = 1;
    
  }
  
  public static final class ShooterConstants {
    public static double kTwoBallRPM = 4350;
    public static double kNoTargetRPM = 4350;
    public static int kShooterBaseRPM = 2500;
    public static double kShooterRPMIncreasePerFoot = 160;
    public static double kShooterDefaultRPM = 3800;
    public static final double kSVolts = 0.05;
    public static final boolean kEncoderReversed = false;
    
    public static final int kShooterMotorPort = 9;
    public static final int kFeederMotorPort = 8;
    
    public static final double kShooterFreeRPS = 5300;
    public static final double kShooterTargetRPS = 4000;
    public static final double kShooterToleranceRPM = 50;
    public static final double kShootTimeSeconds = 4;
    public static final double kShootTimeoutSeconds = 7;
    
   
    public static final double kVVoltSecondsPerRotation =
    // Should have value 12V at free speed...
    12.0 / kShooterFreeRPS;
    
    public static final double kFeederSpeed = 0.5;
  }
  
  public static final class AutoConstants {
    public static final double kAutoTimeoutSeconds = 6;
    public static final double kAutoShootTimeSeconds = 3;
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
  
    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
  
  public static final class OIConstants {
    public static final int kManipulatorControllerPort = 0;
    public static final int kDriverControllerPort1 = 1;
    public static final int kDriverControllerPort2 = 2;
  }


  
}
