package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public final class Constants {
  public static final double kRoot2 = Math.sqrt(2);

  public static final class OperatorConstants {
    public static final int kDriverLeftJoystickPort = 0;
    public static final int kDriverRightJoystickPort = 1;
    public static final int kManipulatorControllerPort = 2;

    public static final double kInputDeadzone = 0.1;

    public static final boolean kApplyCubic = false;
    public static final double kCubicLinearity = 0;

    public static final boolean kApplySin = true;
    public static final double kSinLinearity = 0;

    public static final boolean kLimitTurnSpeed = false;
    public static final double turnSpeedMultiplier = 0.5;

    // The ramp limits are the amount of seconds it should take to go from 0% to 100% speed while shifted to high or low gear
    public static final double kRampLimitLowGearSeconds = 0;
    public static final double kRampLimitHighGearSeconds = 0;

    public static final double kSlowModeMultiplier = 0.3;
    public static final double kLightsTimeoutSeconds = 5;
  }

  public static final class PneumaticsConstants{
    public static final int kPneumaticsHubPort = 22;
    public static final Value kShifterHighSpeed = Value.kReverse;
    public static final Value kShifterLowSpeed = Value.kForward;

    public static final Value kArmRaise = Value.kForward;
    public static final Value kArmLower = Value.kReverse;
  }

  public static final class DriveConstants {
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

    public static final double kMaximumDrivetrainSpeed = 1.0;

    // = measured / input
    public static final double kDriveError = 1.110586;
    public static final double kTurnError = 1.06;
    public static final double kFastRevPerRot = 55d / 544d * kDriveError; // Wheel revolutions per motor rotation
    public static final double kSlowRevPerRot = 11d / 288d * kDriveError;
    
    public static final double kWheelCircumference = 6 * Math.PI;
    public static final double kTurnCircumference = 20 * Math.PI;

    public static final double kTurnAggression = 0.015;
    public static final double kCorrectionAggression = 0.0075;
    public static final double kAllowableHeadingOffset = 0.5d;
    public static final double kMotorStallSpeed = 0.2;
  }

  public static final class ArmConstants {
    public static final int kArmRaiseChannel = 2;
    public static final int kArmLowerChannel = 3;

    public static final int kElbowMotorPort = 5;
    public static final int kElevatorMotor1Port = 6;
    public static final int kElevatorMotor2Port = 7;
    public static final boolean kElevatorMotor1Inverted = true;
    public static final boolean kElevatorMotor2Inverted = true;
    public static final boolean kElbowMotorInverted = true;

    public static final double kElevatorIdleRotations = 1.5;
    public static final double kElevatorExtendRotations = 17.0;
    public static final double kElevatorMaximumSpeed = 0.3;

    // PID Stop thresholds
    public static final double kElevatorStopThreshold = 0.1;
    public static final double kElbowStopThreshold = 0.1;

    public static final double kElbowLowExtendRotations = 14;
    public static final double kElbowMidExtendRotations = 18.0;
    public static final double kElbowHighExtendRotations = 15.0;
    public static final double kElbowIdleExtendRotations = 1;
    public static final double kElbowHighHumanPlayerStationRotations = 10;
    public static final double kElbowAllowedTuning = 5;



    public static enum ScoringPosition {LOW, MID, HIGH};

    // The maximum speed to move to elbow if the elevator is in vs out
    public static final double kElbowRetractedMaximumSpeed = 0.4;
    public static final double kElbowExtendedMaximumSpeed = 0.25;
  }

  public static final class IntakeConstants {
    public static final int kIntakeMotorTopPort = 8;
    public static final int kIntakeMotorBottomPort = 9;

    public static final boolean kIntakeMotorTopReversed = true;
    public static final boolean kIntakeMotorBottomReversed = true;

    public static final double kIntakeInSpeed = 0.5;
    public static final double kIntakeOutSpeedWhenElbowOut = 0.5;
    public static final double kIntakeOutSpeedWhenElbowIn = 1;
  }

  public static final class LightStripConstants {
    public static final int kLightstripPort = 0;
    public static final int kLightstripLength = 60;

    public static final int kOff = 0;
    public static final int kOrange = 1;
    public static final int kYellow = 2;
    public static final int kPurple = 3;
    public static final int kSlowChaser = 4;
    public static final int kFastChaser = 5;
  }
}
