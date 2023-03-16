package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm extends SubsystemBase {

    private final DoubleSolenoid m_piston = new DoubleSolenoid(PneumaticsConstants.kPneumaticsHubPort, PneumaticsModuleType.REVPH, ArmConstants.kArmRaiseChannel, ArmConstants.kArmLowerChannel);

    private final CANSparkMax m_elevator1 = new CANSparkMax(ArmConstants.kElevatorMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_elevator2 = new CANSparkMax(ArmConstants.kElevatorMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_elevator = new MotorControllerGroup(m_elevator1, m_elevator2);
    private final RelativeEncoder m_elevatorEncoder = m_elevator1.getEncoder();

    private final PIDController elevatorPID = new PIDController(0.25, 0, 0);

    private double desiredPosition = ArmConstants.kElevatorIdleRotations;

    private boolean elevatorEncoderResetting;

    private double elevatorEncoderResetStartTime;

    public Arm() {

        m_elevator1.setInverted(ArmConstants.kElevatorMotor1Inverted);
        m_elevator1.setInverted(ArmConstants.kElevatorMotor2Inverted);

        m_elevator1.setIdleMode(IdleMode.kBrake);
        m_elevator2.setIdleMode(IdleMode.kBrake);

        elevatorPID.setTolerance(ArmConstants.kElevatorStopThreshold);
    }

    public boolean getElevatorExtended() {
        if (m_elevatorEncoder.getPosition() > ArmConstants.kElevatorIdleRotations - ArmConstants.kElevatorStopThreshold) {
            return true;
        } else {
            return false;
        }
    }

    public void setElevatorExtended(boolean isExtended) {
        if (getPistonRaised() && isExtended) {
            desiredPosition = (ArmConstants.kElevatorExtendRotations);
        } else {
            desiredPosition = (ArmConstants.kElevatorIdleRotations);
        }
    }

    public boolean getPistonRaised() {
        if (m_piston.get() == PneumaticsConstants.kArmRaise) {
            return true;
        } else {
            return false;
        }
    }

    public void setPistonRaised(boolean isUp) {
        if (!isUp) {
            if (getElevatorExtended()) {
                setElevatorExtended(false);
                }
            m_piston.set(PneumaticsConstants.kArmLower);
        } else if (isUp) {
            m_piston.set(PneumaticsConstants.kArmRaise);
        }
    }

    @Override
    public void periodic() {
        double output = 0;
        if (!elevatorEncoderResetting) {
            output = elevatorPID.calculate(m_elevatorEncoder.getPosition(), desiredPosition);

            output = Math.min(output, 1);
            output = Math.max(output, -1);
            output = output * ArmConstants.kElevatorMaximumSpeed;
        } else {
            double elapsed = System.currentTimeMillis() - elevatorEncoderResetStartTime;
            if (elapsed  > 3000) {
                System.out.println("Arm reset timeout reached");
                elevatorEncoderResetting = false;
                m_elevatorEncoder.setPosition(0);
                output = 0;
            } else if (elapsed > 100) {
                double speed = Math.abs(m_elevatorEncoder.getVelocity());
                System.out.printf("Speed: %.2f\n", speed);
                if (speed < 5) {
                    output = 0;
                    elevatorEncoderResetting = false;
                    m_elevatorEncoder.setPosition(0);
                } else {
                    output = -0.15;
                }
            } else {
                output = -0.15;
            }

        }
        m_elevator.set(output);
    }


    public void init() {
        elevatorEncoderResetting = true;
        elevatorEncoderResetStartTime = System.currentTimeMillis();
        setPistonRaised(true);
    }
}
