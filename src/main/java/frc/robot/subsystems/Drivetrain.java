package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase{
    
    private final CANSparkMax m_leftMotor1 = new CANSparkMax(1, MotorType.kBrushless);
    private final CANSparkMax m_leftMotor2 = new CANSparkMax(2, MotorType.kBrushless);

    private final CANSparkMax m_rightMotor1 = new CANSparkMax(3, MotorType.kBrushless);
    private final CANSparkMax m_rightMotor2 = new CANSparkMax(4, MotorType.kBrushless);

    private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(m_leftMotor1, m_leftMotor2);
    private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(m_rightMotor1, m_rightMotor2);

    private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotors, m_rightMotors);
    private final PneumaticHub m_pneumaticHub = new PneumaticHub();
    private final DoubleSolenoid m_solenoid = new DoubleSolenoid(22, PneumaticsModuleType.REVPH, 0, 1);

    public Drivetrain() {
        m_rightMotors.setInverted(true);

    }
    public void init() {
        m_pneumaticHub.enableCompressorDigital();
        m_solenoid.set(Value.kForward);
     }

    public void tankDrive(double leftSpeed, double rightSpeed, boolean fast) {
        if (fast) {
            m_solenoid.set(Value.kForward);
        } else {
            m_solenoid.set(Value.kReverse);
        }
        m_diffDrive.tankDrive(leftSpeed, rightSpeed);
    }

    @Override
    public void periodic() {}
}
