package frc.robot.subsystems;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PneumaticsConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Arm extends PIDSubsystem{

    private final DoubleSolenoid m_piston = new DoubleSolenoid(PneumaticsConstants.kPneumaticsHubPort, PneumaticsModuleType.REVPH, ArmConstants.kArmRaiseChannel, ArmConstants.kArmLowerChannel);

    private final CANSparkMax m_elevator1 = new CANSparkMax(ArmConstants.kElevatorMotor1Port, MotorType.kBrushless);
    private final CANSparkMax m_elevator2 = new CANSparkMax(ArmConstants.kElevatorMotor2Port, MotorType.kBrushless);
    private final MotorControllerGroup m_elevator = new MotorControllerGroup(m_elevator1, m_elevator2);
    private final RelativeEncoder m_elevatorEncoder = m_elevator1.getEncoder();
    
    
    public Arm() {
        super(new PIDController(0.1,0,0));
        getController().setTolerance(ArmConstants.kElevatorStopThreshold);
        
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

    @Override
    public double getMeasurement() {
        return m_elevatorEncoder.getPosition();
    }

    @Override
    public void useOutput(double output, double setpoint) {
        m_elevator.set(output);
    }

    public boolean getElevatorExtended() {
        if (m_elevatorEncoder.getPosition() > 1) {
            return true;
        } else {
            return false;
        }

    }

    public void setElevatorExtended(boolean isExtended) {
        if (getPistonRaised() && isExtended) {
            setSetpoint(ArmConstants.kElevatorExtendRotations);
        } else {
            setSetpoint(0);
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
        
        if (isUp) {
            if (getElevatorExtended()) {
                setElevatorExtended(false);
            }
            m_piston.set(PneumaticsConstants.kArmRaise);
        } else {
            m_piston.set(PneumaticsConstants.kArmLower);
        }
    }
}
