package frc.robot.subsystems;


import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;

import java.lang.management.OperatingSystemMXBean;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    
    private final CANSparkMax m_leftMotor1 = new CANSparkMax(DriveConstants.kLeftMotorPort1, MotorType.kBrushless);
    private final CANSparkMax m_leftMotor2 = new CANSparkMax(DriveConstants.kLeftMotorPort2, MotorType.kBrushless);
    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);

    private final CANSparkMax m_rightMotor1 = new CANSparkMax(DriveConstants.kRightMotorPort1, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor2 = new CANSparkMax(DriveConstants.kRightMotorPort2, MotorType.kBrushless);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

    // private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors); // Default drivetrain code
    private final PneumaticHub m_pneumaticHub = new PneumaticHub();
    private final DoubleSolenoid m_shifter_solenoid = new DoubleSolenoid(DriveConstants.kShifterPort, PneumaticsModuleType.REVPH, DriveConstants.kShifterHighSpeedChannel, DriveConstants.kShifterLowSpeedChannel);

    public Drivetrain() {
        m_leftMotors.setInverted(DriveConstants.kLeftMotorsInverted);
        m_rightMotors.setInverted(DriveConstants.kRightMotorsInverted);
        m_shifter_solenoid.set(DriveConstants.kShifterLowSpeed);
        m_pneumaticHub.enableCompressorDigital();
    }

    public double applyDeadzone(double speed, double deadzone) {
        if(Math.abs(speed) < deadzone) {
            return 0;
        }
        else {
            return speed * (1 - deadzone);
        }
    }

    public void tankDrive(double leftSpeed, double rightSpeed, boolean fast) {
        if (fast) {
            m_shifter_solenoid.set(DriveConstants.kShifterHighSpeed);
        } else {
            m_shifter_solenoid.set(DriveConstants.kShifterLowSpeed);
        }

        leftSpeed = applyDeadzone(leftSpeed, OperatorConstants.kInputDeadzone);
        rightSpeed = applyDeadzone(rightSpeed, OperatorConstants.kInputDeadzone);
        leftSpeed = (Math.pow(leftSpeed, 3) + (OperatorConstants.kInputLinearity * leftSpeed)) / (1 + OperatorConstants.kInputLinearity);
        rightSpeed = (Math.pow(rightSpeed, 3) + (OperatorConstants.kInputLinearity * rightSpeed)) / (1 + OperatorConstants.kInputLinearity);
        m_leftMotors.set(leftSpeed);
        m_rightMotors.set(rightSpeed);
        
        // m_diffDrive.tankDrive(leftSpeed, rightSpeed); // Default tank drive code
    }
}
