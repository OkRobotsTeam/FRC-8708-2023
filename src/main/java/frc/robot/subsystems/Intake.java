package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class Intake extends SubsystemBase {

    private final CANSparkMax m_intakeTop = new CANSparkMax(IntakeConstants.kIntakeMotorTop, MotorType.kBrushless);
    private final CANSparkMax m_intakeBottom = new CANSparkMax(IntakeConstants.kIntakeMotorBottom, MotorType.kBrushless);
    private final MotorControllerGroup m_intake = new MotorControllerGroup(m_intakeBottom, m_intakeTop);

    public Intake() {
        m_intakeBottom.setInverted(IntakeConstants.kIntakeMotorBottomReversed);
        m_intakeTop.setInverted(IntakeConstants.kIntakeMotorTopReversed);
    }

    public void intakeIn() {
        m_intake.set(IntakeConstants.kMaximumIntakeSpeed);
    }

    public void intakeOut() {
        m_intake.set(-IntakeConstants.kMaximumIntakeSpeed);
    }

}
