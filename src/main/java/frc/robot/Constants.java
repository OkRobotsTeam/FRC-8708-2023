// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class Constants {
  public static class OperatorConstants {
    //HARDWARE CONFIG:
    public static final int kDriverLeftJoystickPort = 0;
    public static final int kDriverRightJoystickPort = 1;
    public static final int kManipulatorControllerPort = 2;
    //SOFTWARE CONFIG:
    public static double kInputDeadzone = 0.1f;
    public static double kInputLinearity = 0.0f;
  }
  public static final class PneumaticsConstants{
    //HARDWARE CONFIG:
    public static final int kPneumaticsHubPort = 22;
    public static final Value kShifterHighSpeed = Value.kForward;
    public static final Value kShifterLowSpeed = Value.kReverse;

    public static final Value kArmRaise = Value.kForward;
    public static final Value kArmLower = Value.kReverse;
  }
  public static class DriveConstants {
    //HARDWARE CONFIG:
    // Left side
    public static final int kLeftMotor1Port = 1;
    public static final int kLeftMotor2Port = 2;
    public static final boolean kLeftMotorsInverted = false;

    // Right side
    public static final int kRightMotor1Port = 3;
    public static final int kRightMotor2Port = 4;
    public static final boolean kRightMotorsInverted = true;

    // Shifter
    public static final int kShifterHighSpeedChannel = 0;
    public static final int kShifterLowSpeedChannel = 1;
    //SOFTWARE CONFIG:
    public static final double kMaximumSpeed = 1.0;
  }
  public final class ArmConstants {
    //HARDWARE CONFIG:
    // Pneumatic channels
    public static final int kArmRaiseChannel = 2;
    public static final int kArmLowerChannel = 3;

    public static final int kElbowMotorPort = 5;
    public static final int kElevatorMotor1Port = 6;
    public static final int kElevatorMotor2Port = 7;

    public static final double kElevatorExtendRotations = 20.0;
    public static final double kMaxElevatorSpeed = 1.0;
  }
}
