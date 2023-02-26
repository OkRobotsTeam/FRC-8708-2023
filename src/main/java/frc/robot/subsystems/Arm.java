package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm extends SubsystemBase{

    private final DoubleSolenoid m_piston = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,2,3);

    private final CANSparkMax m_elbow = new CANSparkMax(5,MotorType.kBrushless);
    private final RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();

    private final CANSparkMax m_elevator1 = new CANSparkMax(6, MotorType.kBrushless);
    private final CANSparkMax m_elevator2 = new CANSparkMax(7, MotorType.kBrushless);
    private final MotorControllerGroup m_elevator = new MotorControllerGroup(m_elevator1, m_elevator2);
    private final RelativeEncoder m_elevatorEncoder = m_elevator1.getEncoder();

    
    public Arm() {}

    public void init() {
        m_elevator1.setIdleMode(IdleMode.kBrake);
        m_elevator2.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void periodic() {}
}
