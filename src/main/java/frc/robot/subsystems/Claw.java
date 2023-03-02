package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class Claw extends SubsystemBase {

    private final CANSparkMax m_intakeTop = new CANSparkMax(ArmConstants.kElevatorMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_intakeBottom = new CANSparkMax(ArmConstants.kElevatorMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_intake = new MotorControllerGroup(m_intakeBottom, m_intakeTop);

    public Claw() {
        m_intakeBottom.setInverted(IntakeConstants.kIntakeMotorBottomReversed);
        m_intakeTop.setInverted(IntakeConstants.kIntakeMotorTopReversed);
    }

    public void setIntakeState(int state) {
        if (state == IntakeConstants.kIntakeDisabled) {
            m_intake.set(0);
        } else if (state == IntakeConstants.kIntakeIn) {
            m_intake.set(IntakeConstants.kMaximumIntakeSpeed);
        } else if (state == IntakeConstants.kIntakeOut) {
            m_intake.set(IntakeConstants.kMaximumIntakeSpeed * -1);
        }
    }

}
