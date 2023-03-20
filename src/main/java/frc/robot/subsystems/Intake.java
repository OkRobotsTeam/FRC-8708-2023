package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_intakeTop = new CANSparkMax(IntakeConstants.kIntakeMotorTopPort, MotorType.kBrushless);
    private final CANSparkMax m_intakeBottom = new CANSparkMax(IntakeConstants.kIntakeMotorBottomPort, MotorType.kBrushless);
    private final MotorControllerGroup m_intake = new MotorControllerGroup(m_intakeBottom, m_intakeTop);

    private Arm m_arm;

    public Intake(Arm arm) {
        m_arm = arm;
        m_intakeBottom.setInverted(IntakeConstants.kIntakeMotorBottomReversed);
        m_intakeTop.setInverted(IntakeConstants.kIntakeMotorTopReversed);
    }

    public void intakeIn() {
        m_intake.set(IntakeConstants.kIntakeInSpeed);
    }

    public void intakeOut() {
        if (m_arm.getElbowExtended()) {
            m_intake.set(-IntakeConstants.kIntakeOutSpeedWhenElbowOut);
        } else {
            m_intake.set(-IntakeConstants.kIntakeOutSpeedWhenElbowIn);
        }
    }

    public void intakeStop() {
        m_intake.set(0);
    }
}
