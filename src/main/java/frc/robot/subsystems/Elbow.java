package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {

    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.kElbowMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();
    private boolean elbowExtended = false;
    private Arm m_arm;

    public Elbow(Arm arm) {
        m_arm = arm;
        m_elbow.setInverted(true);
        m_elbow.setIdleMode(IdleMode.kBrake);
    }

    private void updateElbowSpeed() {
        if (m_arm.getPistonRaised()) {
            if (elbowExtended) {
                if (m_elbowEncoder.getPosition() > ArmConstants.kHighElbowExtendRotations - ArmConstants.kHighElbowStopThreshold) {
                    m_elbow.set(0);
                } else {
                    m_elbow.set(ArmConstants.kHighMaximumElbowSpeed);
                }
            } else {
                if (m_elbowEncoder.getPosition() < ArmConstants.kHighElbowStopThreshold) {
                    m_elbow.set(0);
                } else {
                    m_elbow.set(-ArmConstants.kHighMaximumElbowSpeed);
                }
            }
        } else {
            if (elbowExtended) {
                if (m_elbowEncoder.getPosition() > ArmConstants.kLowElbowExtendRotations - ArmConstants.kLowElbowStopThreshold) {
                    m_elbow.set(0);
                } else {
                    m_elbow.set(ArmConstants.kLowMaximumElbowSpeed);
                }
            } else {
                if (m_elbowEncoder.getPosition() < ArmConstants.kLowElbowStopThreshold) {
                    m_elbow.set(0);
                } else {
                    m_elbow.set(-ArmConstants.kLowMaximumElbowSpeed);
                }
            }
        }
    }

    public void setElbowExtended(boolean isExtended) {
        elbowExtended = isExtended;
    }

    @Override
    public void periodic() {
        updateElbowSpeed();
    }
}
