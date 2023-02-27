package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm extends SubsystemBase{

    private final DoubleSolenoid m_piston = new DoubleSolenoid(PneumaticsModuleType.REVPH, ArmConstants.kArmRaiseChannel, ArmConstants.kArmLowerChannel);

    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.kElbowMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();

    private final CANSparkMax m_elevator1 = new CANSparkMax(ArmConstants.kElevatorMotorPort1, MotorType.kBrushless);
    private final CANSparkMax m_elevator2 = new CANSparkMax(ArmConstants.kElevatorMotorPort2, MotorType.kBrushless);
    private final MotorControllerGroup m_elevator = new MotorControllerGroup(m_elevator1, m_elevator2);
    private final RelativeEncoder m_elevatorEncoder = m_elevator1.getEncoder();

    
    public Arm() {}

    public void init() {
        // Ensure that if we stop sending commands to the elevator it stays put
        m_elevator1.setIdleMode(IdleMode.kBrake);
        m_elevator2.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {}
}
