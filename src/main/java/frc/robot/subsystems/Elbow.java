package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elbow extends SubsystemBase {

    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.kElbowMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();
    private Arm m_arm;

    private double target;


    private final PIDController pid = new PIDController(0.2,0,0);

    public Elbow(Arm arm) {
        m_arm = arm;
        m_elbow.setInverted(true);
        m_elbow.setIdleMode(IdleMode.kBrake);
        m_elbowEncoder.setPosition(0);
        pid.setTolerance(1);
    }

    public void setElbowExtended(boolean isExtended) {
        if (isExtended) {
            if (m_arm.getPistonRaised()) {
                target = (ArmConstants.kHighElbowExtendRotations);
            } else {
                target = (ArmConstants.kLowElbowExtendRotations);
            }
        } else {
            target = (0);
        }
    }

    public void incTarget() {
        target += 0.5;
    }

    public void decTarget() {
        target -= 0.5;
    }

    @Override
    public void periodic() {
        double output = pid.calculate(m_elbowEncoder.getPosition(),target);
        m_elbow.set(0.1*output);
        //System.out.println(output);
    }
}
