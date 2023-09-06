package frc.robot.subsystems;

import frc.robot.Constants.IntakeConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

public class Intake extends SubsystemBase {

    private final CANSparkMax m_intakeTop = new CANSparkMax(IntakeConstants.kIntakeMotorTopPort, MotorType.kBrushless);
    private final CANSparkMax m_intakeBottom = new CANSparkMax(IntakeConstants.kIntakeMotorBottomPort, MotorType.kBrushless);
    private final RelativeEncoder m_intakeBottomEncoder = m_intakeBottom.getEncoder();
    private final RelativeEncoder m_intakeTopEncoder = m_intakeBottom.getEncoder();
    private final MotorControllerGroup m_intake = new MotorControllerGroup(m_intakeBottom, m_intakeTop);

    private double intakeInStartTime = 0;

    private Arm m_arm;


    public Intake(Arm arm) {
        m_arm = arm;
        m_intakeBottom.setInverted(IntakeConstants.kIntakeMotorBottomReversed);
        m_intakeTop.setInverted(IntakeConstants.kIntakeMotorTopReversed);
        m_intakeBottomEncoder.setPositionConversionFactor(1d/9d);
        m_intakeTopEncoder.setPositionConversionFactor(1d/9d);
        m_intakeBottomEncoder.setVelocityConversionFactor(1d/9d);
        m_intakeTopEncoder.setVelocityConversionFactor(1d/9d);
    }

    public void intakeIn() {
        intakeInStartTime = System.currentTimeMillis();
        m_intake.set(IntakeConstants.kIntakeInSpeed);
    }

    public void intakeOut() {
        if (m_arm.getElbowExtended()) {
            m_intake.set(-IntakeConstants.kIntakeOutSpeedWhenElbowOut);
        } else {
            m_intake.set(-IntakeConstants.kIntakeOutSpeedWhenElbowIn);
        }
    }

    public void intakeStop() {
        m_intake.set(0);
    }

    public void checkPickedUp(GenericEntry objedtInIntake) {
        // Don't take reading until 500 milliseconds after the motors start
        if (System.currentTimeMillis() - intakeInStartTime > IntakeConstants.kIntakeMotorStartupTimeMSEC) {
            if (m_intake.get() < 0) {
                // The intake is running in reverse, clear the status light
                // Display a white square on the dashboard
                objedtInIntake.setBoolean(false);
            } else if ((m_intakeTop.getOutputCurrent() > IntakeConstants.kHoldingObjectCurrent) | (m_intakeBottom.getOutputCurrent() > IntakeConstants.kHoldingObjectCurrent)) {
                // The intake is pulling in and the current is higher than the threshhold
                // This means we have a game object
                // Display a green square on the dashboard
                objedtInIntake.setBoolean(true);
                intakeStop();
            } else if (m_intake.get() > 0) {
                // The intake is pulling in but the RPM is still higher than the threshhold
                // Display a red square on the dashboard
                objedtInIntake.setBoolean(false);
            }
        }
    }
}
