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

    private double target = ArmConstants.kIdleElbowExtendRotations;

    private final PIDController pid = new PIDController(0.15, 0, 0);

    public Elbow(Arm arm) {
        m_arm = arm;
        m_elbow.setInverted(ArmConstants.kElbowMotorInverted);
        m_elbow.setIdleMode(IdleMode.kBrake);
        m_elbowEncoder.setPosition(0); // Reset relative encoder
        pid.setTolerance(ArmConstants.kElbowStopThreshold);
    }

    public void setElbowExtended(boolean isExtended) {
        if (isExtended) {
            if (m_arm.getPistonRaised()) {
                target = (ArmConstants.kHighElbowExtendRotations);
            } else {
                target = (ArmConstants.kLowElbowExtendRotations);
            }
        } else {
            target = ArmConstants.kIdleElbowExtendRotations;
        }
    }

    public void tuneTarget(double amount) {
        target += amount;

        // Clamp the target betwwen 0 and the default extended rotations + 5
        target = Math.max(0, target);
        if (m_arm.getPistonRaised()) {
            target = Math.min(target, ArmConstants.kHighElbowExtendRotations + 5);
        } else {
            target = Math.min(target, ArmConstants.kLowElbowExtendRotations + 5);
        }
    }

    @Override
    public void periodic() {
        double output = pid.calculate(m_elbowEncoder.getPosition(), target);
        // Clamp the pid output between 0.4 and -0.4
        output = Math.min(output, 1);
        output = Math.max(output, -1);
        output = output * 0.4;
        m_elbow.set(output);
    }

    public void teleopInit() {
        m_elbowEncoder.setPosition(0);
        target = ArmConstants.kIdleElbowExtendRotations;
    }
}
