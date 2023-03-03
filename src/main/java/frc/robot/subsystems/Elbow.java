package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;

public class Elbow extends PIDSubsystem {

    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.kElbowMotorPort, MotorType.kBrushless);
    private final RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();
    private Arm m_arm;

    public Elbow(Arm arm) {
        super(new PIDController(0.1,0,0));
        m_arm = arm;
        m_elbow.setInverted(true);
        m_elbow.setIdleMode(IdleMode.kBrake);
    }

    @Override
    public void useOutput(double output, double setpoint) {
        m_elbow.set(output);
    }

    @Override
    public double getMeasurement() {
        return m_elbowEncoder.getPosition();
    }

    public void setElbowExtended(boolean isExtended) {
        if (isExtended) {
            if (m_arm.getPistonRaised()) {
                setSetpoint(ArmConstants.kHighElbowExtendRotations);
            } else {
                setSetpoint(ArmConstants.kLowElbowExtendRotations);
            }
        } else {
            setSetpoint(0);
        }
    }
}
