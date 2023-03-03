package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm extends SubsystemBase{

    private final DoubleSolenoid m_piston = new DoubleSolenoid(PneumaticsConstants.kPneumaticsHubPort, PneumaticsModuleType.REVPH, ArmConstants.kArmRaiseChannel, ArmConstants.kArmLowerChannel);

    private final CANSparkMax m_elevator1 = new CANSparkMax(ArmConstants.kElevatorMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_elevator2 = new CANSparkMax(ArmConstants.kElevatorMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_elevator = new MotorControllerGroup(m_elevator1, m_elevator2);
    private final RelativeEncoder m_elevatorEncoder = m_elevator1.getEncoder();
    
    private boolean elevatorExtended = false;
    
    public Arm() {
        m_elevator1.setInverted(true);
        m_elevator1.setInverted(true);

        m_elevator1.setIdleMode(IdleMode.kBrake);
        m_elevator2.setIdleMode(IdleMode.kBrake);

        m_elevator1.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_elevator1.enableSoftLimit(SoftLimitDirection.kReverse, true);
        m_elevator2.enableSoftLimit(SoftLimitDirection.kForward, true);
        m_elevator2.enableSoftLimit(SoftLimitDirection.kReverse, true);

        m_elevator1.setSoftLimit(SoftLimitDirection.kForward, (float)ArmConstants.kElevatorExtendRotations);
        m_elevator1.setSoftLimit(SoftLimitDirection.kReverse, 0.0f);
        m_elevator2.setSoftLimit(SoftLimitDirection.kForward, (float)ArmConstants.kElevatorExtendRotations);
        m_elevator2.setSoftLimit(SoftLimitDirection.kReverse, 0.0f);

        m_elevatorEncoder.setPosition(0);
    }

    private void updateElevatorSpeed() {
        if (elevatorExtended) {
            if (m_elevatorEncoder.getPosition() > ArmConstants.kElevatorExtendRotations - ArmConstants.kElevatorStopThreshold) {
                m_elevator.set(0);
            } else {
                m_elevator.set(ArmConstants.kMaximumElevatorSpeed);
            }
        } else {
            if (m_elevatorEncoder.getPosition() < ArmConstants.kElevatorStopThreshold) {
                m_elevator.set(0);
            } else {
                m_elevator.set(-ArmConstants.kMaximumElevatorSpeed);
            }
        }
    }

    public boolean getElevatorExtended() {
        if (m_elevatorEncoder.getPosition() > 1) {
            return true;
        } else {
            return false;
        }
    }

    public void setElevatorExtended(boolean isExtended) {
        if (getPistonRaised()) {
            elevatorExtended = isExtended;
        } else {
            elevatorExtended = false;
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
        if (getElevatorExtended()) {
            setElevatorExtended(false);
        }
        if (isUp) {
            m_piston.set(PneumaticsConstants.kArmRaise);
        } else {
            m_piston.set(PneumaticsConstants.kArmLower);
        }
    }
    
    public void SetElevatorMotors(double speed) {
        m_elevator.set(speed);
        System.out.println("Elevator Encoder: " + m_elevatorEncoder.getPosition());

    }


    @Override
    public void periodic() {
        updateElevatorSpeed();
    }
}
