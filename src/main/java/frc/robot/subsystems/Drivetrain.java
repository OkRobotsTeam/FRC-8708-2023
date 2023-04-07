package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PneumaticsConstants;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Drivetrain extends SubsystemBase {
    private final CANSparkMax m_leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    public final RelativeEncoder m_leftEncoder = m_leftMotor1.getEncoder();

    private final CANSparkMax m_rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
    public final RelativeEncoder m_rightEncoder = m_rightMotor1.getEncoder();

    private final PneumaticHub m_pneumaticHub = new PneumaticHub();
    private final DoubleSolenoid m_shifter_solenoid = new DoubleSolenoid(PneumaticsConstants.kPneumaticsHubPort,
            PneumaticsModuleType.REVPH, DriveConstants.kShifterHighSpeedChannel,
            DriveConstants.kShifterLowSpeedChannel);

    private boolean previousFast;

    private double prevLeftPos = 0;
    private double prevRightPos = 0;

    private NetworkTableEntry txEntry;
    private NetworkTableEntry tyEntry;

    public ADIS16470_IMU gyro = new ADIS16470_IMU();

    public DifferentialDriveOdometry drivetrain = new DifferentialDriveOdometry(new Rotation2d(gyro.getAngle()), 0,0);

    public Drivetrain() {
        // Invert motor groups according to the constants
        m_leftMotors.setInverted(DriveConstants.kLeftMotorsInverted);
        m_rightMotors.setInverted(DriveConstants.kRightMotorsInverted);

        // DOESN'T WORK
        //m_leftEncoder.setInverted(DriveConstants.kLeftMotorsInverted);
        //m_rightEncoder.setInverted(DriveConstants.kRightMotorsInverted);

        // previousFast is a boolean value holding whether the fast argument was true
        // last time we checked
        previousFast = false;
        m_leftMotor1.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);
        m_leftMotor2.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);
        m_rightMotor1.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);
        m_rightMotor2.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);
        // Shift to low gear by default
        m_shifter_solenoid.set(PneumaticsConstants.kShifterLowSpeed);
        // Enable the compressor using a digital sensor to stop it when it gets to
        // pressure
        m_pneumaticHub.enableCompressorDigital();

        txEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx");
        tyEntry = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty");

    }

    public void setBrakeMode(boolean brake) {
        if (brake) {
            m_leftMotor1.setIdleMode(IdleMode.kBrake);
            m_leftMotor2.setIdleMode(IdleMode.kBrake);
            m_rightMotor1.setIdleMode(IdleMode.kBrake);
            m_rightMotor2.setIdleMode(IdleMode.kBrake);
        } else {
            m_leftMotor1.setIdleMode(IdleMode.kCoast);
            m_leftMotor2.setIdleMode(IdleMode.kCoast);
            m_rightMotor1.setIdleMode(IdleMode.kCoast);
            m_rightMotor2.setIdleMode(IdleMode.kCoast);
        }
    }

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    }

    public double getRightEncoder() {
        return -m_rightEncoder.getPosition();
    }

    public double getAvgEncoder() {
        return (m_leftEncoder.getPosition() - m_rightEncoder.getPosition()) / 2;
    }

    public double applyDeadzone(double speed, double deadzone) {
        if (Math.abs(speed) < deadzone) {
            return 0;
        } else {
            return (speed - Math.copySign(deadzone, speed)) / (1 - deadzone); // Preserve a "live" zone of 0.0-1.0
        }
    }
    
    // Apply a cubic function to the input with the passed linearity
    public double applyCubic(double speed, double linearity) {
        return (Math.pow(speed, 3) + (linearity * speed)) / (1 + linearity); 
    }

    // https://www.desmos.com/calculator/ibahlh6ezh

    private double snap(double angle, double strength) {
        return strength * Math.sin(4 * angle) / 4 + angle;
    }

    public void snapToClosestDirection(double leftSpeed, double rightSpeed) {
        double r = Math.sqrt(Math.pow(leftSpeed, 2) + Math.pow(rightSpeed, 2));
        double a = Math.atan2(rightSpeed, leftSpeed);
        a = snap(a, r / Constants.kRoot2);
        double newLeft = r * Math.cos(a);
        double newRight = r * Math.sin(a);
        m_leftMotors.set(newLeft);
        m_rightMotors.set(newRight);
    }

    public double applySin(double speed) {
        return speed + (1 - OperatorConstants.kSinLinearity) * Math.sin(2 * Math.PI * speed) / 2 / Math.PI;
    }

    public void tankDriveRaw(double leftSpeed, double rightSpeed, boolean fast) {
        leftSpeed = Math.min(1,Math.max(-1,leftSpeed));
        rightSpeed = Math.min(1,Math.max(-1,rightSpeed)); 
        if (fast != previousFast) {
            System.out.println("SHIFTING");
            if (fast) {
                m_shifter_solenoid.set(PneumaticsConstants.kShifterHighSpeed);
                m_leftMotor1.setOpenLoopRampRate(OperatorConstants.kRampLimitHighGearSeconds);
                m_leftMotor2.setOpenLoopRampRate(OperatorConstants.kRampLimitHighGearSeconds);
                m_rightMotor1.setOpenLoopRampRate(OperatorConstants.kRampLimitHighGearSeconds);
                m_rightMotor2.setOpenLoopRampRate(OperatorConstants.kRampLimitHighGearSeconds);
            } else {
                m_shifter_solenoid.set(PneumaticsConstants.kShifterLowSpeed);
                m_leftMotor1.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);
                m_leftMotor2.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);
                m_rightMotor1.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);
                m_rightMotor2.setOpenLoopRampRate(OperatorConstants.kRampLimitLowGearSeconds);

            }
            previousFast = fast;
        }
        m_leftMotors.set(leftSpeed);
        m_rightMotors.set(rightSpeed);
    }

    public void setRampRate(double rampRate) {
        m_leftMotor1.setOpenLoopRampRate(rampRate);
        m_leftMotor2.setOpenLoopRampRate(rampRate);
        m_rightMotor1.setOpenLoopRampRate(rampRate);
        m_rightMotor2.setOpenLoopRampRate(rampRate);
    }

    public void tankDrive(double leftController, double rightController, boolean fast, boolean slow_brake, boolean slow, boolean auto_correct) {
        double leftSpeed;
        double rightSpeed;
        // Only update the pneumatics if its state changed from the previous state
        if (fast != previousFast) {
            if (fast) {
                m_shifter_solenoid.set(PneumaticsConstants.kShifterHighSpeed);
            } else {
                m_shifter_solenoid.set(PneumaticsConstants.kShifterLowSpeed);
            }
            previousFast = fast;
        }

        // Apply a deadzone to the motor speeds
        leftController = applyDeadzone(leftController, OperatorConstants.kInputDeadzone);
        rightController = applyDeadzone(rightController, OperatorConstants.kInputDeadzone);
        
        leftSpeed = leftController;
        rightSpeed = rightController;

        if (OperatorConstants.kApplyCubic) {
            leftController = applyCubic(leftController, OperatorConstants.kCubicLinearity);
            rightController = applyCubic(rightController, OperatorConstants.kCubicLinearity);
        } else if (OperatorConstants.kApplySin) {
            leftController = applySin(leftController);
            rightController = applySin(rightController);
        }
        if (OperatorConstants.kLimitTurnSpeed) {
            double forwardSpeed = (leftController + rightController) / 2;
            if (forwardSpeed > OperatorConstants.kLimitTurnSpeedThreshhold) {
            double turnSpeed = (leftController - rightController);
                turnSpeed *= OperatorConstants.turnSpeedMultiplier;
                leftSpeed = forwardSpeed + turnSpeed;
                rightSpeed = forwardSpeed - turnSpeed;
            }
        }

        if (auto_correct) {
            double tx = txEntry.getDouble(0);
            double ty = tyEntry.getDouble(0);
            double z = DriveConstants.kLimelightOffsetYInches / Math.tan(Math.toRadians(ty));
            double targetTx = Math.atan(z / DriveConstants.kLimelightOffsetXInches);
            double deltaHeading = (tx - targetTx) + 4;
            System.out.println("Distance ahead: " + z + ", Cube angle: " + tx + ", Cube degrees off: " + deltaHeading);
            leftSpeed += (-deltaHeading * 0.015);
            rightSpeed += (deltaHeading * 0.015);
        }

        
        if (slow_brake) {
            m_leftMotors.set(leftSpeed * OperatorConstants.kSlowModeMultiplier * DriveConstants.kMaximumDrivetrainSpeed);
            m_rightMotors.set(rightSpeed * OperatorConstants.kSlowModeMultiplier * DriveConstants.kMaximumDrivetrainSpeed);
        } else if (slow) {
            m_leftMotors.set(leftSpeed * OperatorConstants.kSlowModeMultiplier * DriveConstants.kMaximumDrivetrainSpeed);
            m_rightMotors.set(rightSpeed * OperatorConstants.kSlowModeMultiplier * DriveConstants.kMaximumDrivetrainSpeed);
        }
        else {
            m_leftMotors.set(leftSpeed * DriveConstants.kMaximumDrivetrainSpeed);
            m_rightMotors.set(rightSpeed * DriveConstants.kMaximumDrivetrainSpeed);
        }

        if (auto_correct || slow_brake) {
            setBrakeMode(true);
        } else {
            setBrakeMode(false);
        }
    }

    private double revToMeters(double rev){
        if (previousFast){
            return rev * DriveConstants.kFastRevPerRot * DriveConstants.kWheelCircumference * 0.0254d;
        } else {
            return rev * DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference * 0.0254d;
        }
    }
    @Override
    public void periodic() {
        double currentLeftPos = m_leftEncoder.getPosition();
        double currentRightPos = m_leftEncoder.getPosition();
        drivetrain.update(new Rotation2d(gyro.getAngle()),
            revToMeters(currentLeftPos - prevLeftPos),
            revToMeters(currentRightPos - prevRightPos)    
        );
        prevLeftPos = currentLeftPos;
        prevRightPos = currentRightPos;
        System.out.println(m_shifter_solenoid.get()==Value.kOff);
    }
}