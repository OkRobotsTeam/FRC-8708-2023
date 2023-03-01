package frc.robot.subsystems;


import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
//import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    
    private final CANSparkMax m_leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    public final RelativeEncoder m_leftEncoder = m_leftMotor1.getEncoder();

    private final CANSparkMax m_rightMotor1 = new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor2 = new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);
    public final RelativeEncoder m_rightEncoder = m_rightMotor1.getEncoder();

    private final PneumaticHub m_pneumaticHub = new PneumaticHub();
    private final DoubleSolenoid m_shifter_solenoid = new DoubleSolenoid(PneumaticsConstants.kPneumaticsHubPort, PneumaticsModuleType.REVPH, DriveConstants.kShifterHighSpeedChannel, DriveConstants.kShifterLowSpeedChannel);

    //public final Gyro m_gyro = new ADXRS450_Gyro();

    private boolean previousFast;

    public Drivetrain() {
        // Invert motor groups according to the constants
        m_leftMotors.setInverted(DriveConstants.kLeftMotorsInverted);
        m_rightMotors.setInverted(DriveConstants.kRightMotorsInverted);
        // previousFast is a boolean value holding whether the fast trigger was clicked last time we checked
        previousFast = false;
        // Shift to low gear by default
        m_shifter_solenoid.set(PneumaticsConstants.kShifterLowSpeed);
        // Enable the compressor using a digital sensor to stop it when it gets to pressure
        m_pneumaticHub.enableCompressorDigital();
    }

    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    public double getLeftEncoder() {
        return m_leftEncoder.getPosition();
    }
    public double getRightEncoder() {
        return m_rightEncoder.getPosition();
    }
    public double getAvgEncoder() {
        return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition())/2;
    }

    public double applyDeadzone(double speed, double deadzone) {
        if(Math.abs(speed) < deadzone) {
            return 0;
        }
        else {
            return (speed - deadzone) / (1 - deadzone); // Preserve a "live" zone of 0.0-1.0
        }
    }

    public double applyCubic(double speed, double linearity) {
        return (Math.pow(speed, 3) + (linearity * speed)) / (1 + linearity); // Apply a cubic function to the input with the passed linearity
    }

    public double applySquare(double speed, double linearity) {
        return (speed * Math.abs(speed) + (linearity * speed))/ (1+linearity);
    }

    public void scaleDifference(double leftSpeed, double rightSpeed) {
        double average = (leftSpeed + rightSpeed) / 2.0;
        double difference = (leftSpeed - rightSpeed) / 2.0;
        double scaledDifference = applySquare(difference,0.1);
        m_leftMotors.set(average + scaledDifference);
        m_rightMotors.set(average - scaledDifference);
    }



    public void tankDrive(double leftSpeed, double rightSpeed, boolean fast) {
        // Only update the pneumatics state if it changed from its last state
        if (fast != previousFast){
            if (fast) {
                m_shifter_solenoid.set(PneumaticsConstants.kShifterHighSpeed);
            } else {
                m_shifter_solenoid.set(PneumaticsConstants.kShifterLowSpeed);
            }
            previousFast = fast;
        }


        // leftSpeed and rightSpeed are doubles from -1.0 to 1.0
        // Apply a deadzone to the motor speeds
        leftSpeed = applyDeadzone(leftSpeed, OperatorConstants.kInputDeadzone);
        rightSpeed = applyDeadzone(rightSpeed, OperatorConstants.kInputDeadzone);
        // Apply a cubic function to the motor speeds
        leftSpeed = applyCubic(leftSpeed, OperatorConstants.kInputLinearity);
        rightSpeed = applyCubic(rightSpeed, OperatorConstants.kInputLinearity);
        // Send the values to the motors
        m_leftMotors.set(leftSpeed * DriveConstants.kMaximumDrivetrainSpeed);
        m_rightMotors.set(rightSpeed * DriveConstants.kMaximumDrivetrainSpeed);
        
    }
}
