package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {

    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.kElbowMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();
    private Arm m_arm;

    private double target = ArmConstants.kElbowIdleExtendRotations;

    private final PIDController elbowPID = new PIDController(0.15, 0, 0);

    public Elbow(Arm arm) {
        m_arm = arm;
        m_elbow.setInverted(ArmConstants.kElbowMotorInverted);
        m_elbow.setIdleMode(IdleMode.kBrake);
        elbowPID.setTolerance(ArmConstants.kElbowStopThreshold);
    }

    public void setElbowExtended(boolean isExtended) {
        if (isExtended) {
            if (m_arm.getPistonRaised()) {
                if (m_arm.getElevatorExtended()) {
                    target = (ArmConstants.kElbowHighExtendRotations);
                } else {
                    target = (ArmConstants.kElbowMidExtendRotations);
                }
            } else {
                target = (ArmConstants.kElbowLowExtendRotations);
            }
        } else {
            target = ArmConstants.kElbowIdleExtendRotations;
        }
    }

    public void manualAdjustTarget(double amount) {
        target += amount;
        target = Math.max(ArmConstants.kElbowIdleExtendRotations, target);
        if (m_arm.getPistonRaised()) {
            target = Math.min(target, ArmConstants.kElbowMidExtendRotations + ArmConstants.kElbowAllowedTuning);
        } else {
            target = Math.min(target, ArmConstants.kElbowLowExtendRotations + ArmConstants.kElbowAllowedTuning);
        }
    }

    @Override
    public void periodic() {
        double output = elbowPID.calculate(m_elbowEncoder.getPosition(), target);
        output = Math.min(output, 1);
        output = Math.max(output, -1);
        if (m_arm.getElevatorExtended()) {
            output = output * ArmConstants.kElbowExtendedMaximumSpeed;
        } else {
            output = output * ArmConstants.kElbowRetractedMaximumSpeed;
        }
        m_elbow.set(output);
    }

    public void init() {
        m_elbowEncoder.setPosition(0);
        target = ArmConstants.kElbowIdleExtendRotations;
    }
}
