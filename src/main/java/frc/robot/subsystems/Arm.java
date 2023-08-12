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
    private final CANSparkMax m_elbow = new CANSparkMax(ArmConstants.kElbowMotorPort, MotorType.kBrushless);
    private final MotorControllerGroup m_elevator = new MotorControllerGroup(m_elevator1, m_elevator2);
    private final RelativeEncoder m_elevatorEncoder = m_elevator1.getEncoder();
    private final RelativeEncoder m_elbowEncoder = m_elbow.getEncoder();

    private final PIDController elevatorPID = new PIDController(0.3, 0, 0);
    private final PIDController elbowPID = new PIDController(0.15, 0, 0);

    public double elevatorDesiredPosition = ArmConstants.kElevatorIdleRotations;
    public double elbowDesiredPosition = ArmConstants.kElbowIdleExtendRotations;

    private boolean elevatorEncoderResetting;
    private double elevatorEncoderResetStartTime;

    private boolean calibrated = false;

    public Arm() {
        m_elevator1.setInverted(ArmConstants.kElevatorMotor1Inverted);
        m_elevator1.setInverted(ArmConstants.kElevatorMotor2Inverted);
        m_elbow.setInverted(ArmConstants.kElbowMotorInverted);

        m_elevator1.setIdleMode(IdleMode.kBrake);
        m_elevator2.setIdleMode(IdleMode.kBrake);
        m_elbow.setIdleMode(IdleMode.kBrake);

        elevatorPID.setTolerance(ArmConstants.kElevatorStopThreshold);
        elbowPID.setTolerance(ArmConstants.kElbowStopThreshold);
    }

    public void setElbowExtended(boolean isExtended) {
        if (isExtended) {
            if (getPistonRaised()) {
                if (getElevatorExtended()) {
                    elbowDesiredPosition = (ArmConstants.kElbowHighExtendRotations);
                } else {
                    elbowDesiredPosition = (ArmConstants.kElbowMidExtendRotations);
                }
            } else {
                elbowDesiredPosition = (ArmConstants.kElbowLowExtendRotations);
            }
        } else {
            elbowDesiredPosition = ArmConstants.kElbowIdleExtendRotations;
        }
    }

    public void toggleElbowExtended() {
        if (getElbowExtended()) {
            elbowDesiredPosition = ArmConstants.kElbowIdleExtendRotations;
        } else {
            if (getPistonRaised()) {
                if (getElevatorExtended()) {
                    elbowDesiredPosition = (ArmConstants.kElbowHighExtendRotations);
                } else {
                    elbowDesiredPosition = (ArmConstants.kElbowMidExtendRotations);
                }
            } else {
                elbowDesiredPosition = (ArmConstants.kElbowLowExtendRotations);
            }
        }
    }

    public boolean getElbowExtended() {
        if (m_elbowEncoder.getPosition() > ArmConstants.kElbowIdleExtendRotations + ArmConstants.kElbowStopThreshold + 4) {
            return true;
        } else {
            return false;
        }
    }

    public void manualAdjustTarget(double amount) {
        elbowDesiredPosition += amount;
        elbowDesiredPosition = Math.max(ArmConstants.kElbowIdleExtendRotations, elbowDesiredPosition);
        if (getPistonRaised()) {
            elbowDesiredPosition = Math.min(elbowDesiredPosition, ArmConstants.kElbowMidExtendRotations + ArmConstants.kElbowAllowedTuning);
        } else {
            elbowDesiredPosition = Math.min(elbowDesiredPosition, ArmConstants.kElbowLowExtendRotations + ArmConstants.kElbowAllowedTuning);
        }
    }

    public boolean getElevatorExtended() {
        if (m_elevatorEncoder.getPosition() > ArmConstants.kElevatorIdleRotations) {
            return true;
        } else {
            return false;
        }
    }

    public void setElevatorExtended(boolean extended) {
        if (getPistonRaised() && extended) {
            elevatorDesiredPosition = (ArmConstants.kElevatorExtendRotations);
        } else {
            elevatorDesiredPosition = (ArmConstants.kElevatorIdleRotations);
        }
    }

    public double getElbowPosition() {
        return(m_elbowEncoder.getPosition());
    }

    public boolean getPistonRaised() {
        if (m_piston.get() == PneumaticsConstants.kArmRaise) {
            return true;
        } else {
            return false;
        }
    }

    public void setPistonRaised(boolean raised) {
        if (!raised) {
            if (getElevatorExtended()) {
                setElevatorExtended(false);
            }
            if (m_elbowEncoder.getPosition() > ArmConstants.kElbowIdleExtendRotations + ArmConstants.kElbowStopThreshold) {
                setElbowExtended(false);
            }
            m_piston.set(PneumaticsConstants.kArmLower);
            //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setInteger(3);
            
        } else if (raised) {
            m_piston.set(PneumaticsConstants.kArmRaise);
            //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setInteger(1);
        }
        
    }

    @Override
    public void periodic() {
        double elevatorPower = 0; // Default value should be 0
        if (!elevatorEncoderResetting) {
            // The elevator is not calibrating, run a PID to try to get to the specified position
            elevatorPower = elevatorPID.calculate(m_elevatorEncoder.getPosition(), elevatorDesiredPosition);

            elevatorPower = Math.min(elevatorPower, 1);
            elevatorPower = Math.max(elevatorPower, -1);
            elevatorPower = elevatorPower * ArmConstants.kElevatorMaximumSpeed;
        } else {
            double timeElapsed = System.currentTimeMillis() - elevatorEncoderResetStartTime;
            if (timeElapsed  > 3000) {
                // The arm has been moving for over 3000 msec, stop blocking the PID and throw a message to the console
                System.out.println("WARNING: Arm reset timeout of 3000 msec exceeded!");
                m_elevatorEncoder.setPosition(0);
                elevatorEncoderResetting = false;
            } else if (timeElapsed > 100) {
                // The motor has had time to get up to speed, start checking if it has hit something
                double speed = Math.abs(m_elevatorEncoder.getVelocity());
                if (speed < 5) {
                    // The eleavtor has hit the end of its track, recalibrate the encoder, and break the loop
                    m_elevatorEncoder.setPosition(0);
                    elevatorEncoderResetting = false;
                } else {
                    // The elevator is resetting but has not yet hit the end, keep going
                    elevatorPower = -0.15;
                }
            } else {
                // The motor started less than 150 msec ago and may not be up to speed, keep going without checking its speed
                elevatorPower = -0.15;
            }

        }
        double elbowPower = elbowPID.calculate(m_elbowEncoder.getPosition(), elbowDesiredPosition);
        elbowPower = Math.min(elbowPower, 1);
        elbowPower = Math.max(elbowPower, -1);
        if (getElevatorExtended()) {
            elbowPower = elbowPower * ArmConstants.kElbowExtendedMaximumSpeed;
        } else {
            elbowPower = elbowPower * ArmConstants.kElbowRetractedMaximumSpeed;
        }
        m_elbow.set(elbowPower);

        m_elevator.set(elevatorPower);
    }


    public void init() {
        if (!calibrated) {
            elevatorEncoderResetting = true;
            elevatorEncoderResetStartTime = System.currentTimeMillis();
            m_elbowEncoder.setPosition(0);
            elbowDesiredPosition = ArmConstants.kElbowIdleExtendRotations;
            setPistonRaised(true);
            calibrated = true;
        }
    }

    public void setElbowSetpoint(double setpoint) {
        elbowDesiredPosition = setpoint;
    }

    public void moveToScoringPosition(ArmConstants.ScoringPosition position) {
        if (position == ArmConstants.ScoringPosition.LOW) {
            setElbowExtended(true);
            setPistonRaised(false);
            setElevatorExtended(false);
        } else if (position == ArmConstants.ScoringPosition.MID) {
            setElevatorExtended(false);
            elbowDesiredPosition = (ArmConstants.kElbowHighExtendRotations);
        } else if (position == ArmConstants.ScoringPosition.HIGH) {
            setElevatorExtended(true);
            elbowDesiredPosition = (ArmConstants.kElbowHighExtendRotations);
            // CommandScheduler.getInstance().schedule(null);
        //     elbowPID.atSetpoint().onTrue(
        //         new InstantCommand(
        //         () -> m_drivetrain.setBrakeMode(true), m_drivetrain));
        }
    }
}
