// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverLeftJoystickPort = 0;
    public static final int kDriverRightJoystickPort = 1;

    public static float kInputDeadzone = 0.1f;
    public static float kInputLinearity = 0.0f;
  }
  public static class DriveConstants {
    public static final int kLeftMotorPort1 = 1;
    public static final int kLeftMotorPort2 = 2;
    public static final boolean kLeftMotorsInverted = false;

    public static final int kRightMotorPort1 = 3;
    public static final int kRightMotorPort2 = 4;
    public static final boolean kRightMotorsInverted = true;

    public static final int kShifterPort = 22;
    public static final int kShifterHighSpeedChannel = 0;
    public static final int kShifterLowSpeedChannel = 1;

    public static final Value kShifterHighSpeed = Value.kForward;
    public static final Value kShifterLowSpeed = Value.kReverse;
  }
}
