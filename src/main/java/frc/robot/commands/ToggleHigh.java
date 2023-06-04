package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class ToggleHigh extends CommandBase {

    private Arm m_arm;
    private double startTime = -1;
    private boolean pistonDone = false;
    private boolean elbowDone = false;
    private boolean exit = false;

    public ToggleHigh(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        exit = false;
        startTime = System.currentTimeMillis();
        elbowDone = m_arm.getElbowExtended();
        pistonDone = m_arm.getPistonRaised();
        System.out.println(elbowDone);
        System.out.println(pistonDone);
        if (elbowDone && pistonDone) {
            System.out.println("Folding");
            m_arm.setElbowExtended(false);
            m_arm.elevatorDesiredPosition = Constants.ArmConstants.kElevatorIdleRotations;
            exit = true;
            
        } else {
            System.out.println("Extending");
            if (pistonDone) {
                m_arm.setElbowSetpoint(ArmConstants.kElbowHighExtendRotations);
            } else {
                m_arm.setPistonRaised(true);
            }
        }
    }

    @Override
    public boolean isFinished() {
        double elbowPosition = m_arm.getElbowPosition();
        elbowDone = (Math.abs(elbowPosition - ArmConstants.kElbowHighExtendRotations) < 2);
        if (startTime > 0 && System.currentTimeMillis() - startTime > 3000) {
            System.out.println("MoveToHigh terminated by timeout");
            return true;
        }
        if (exit) {
            System.out.println("EXIT = TRUE");
            return true;
        }
        if (pistonDone && elbowDone) {
            System.out.println("PISTON AND ELBOW DONE; EXITING");
            m_arm.elevatorDesiredPosition = Constants.ArmConstants.kElevatorExtendRotations;
            return true;
        }

        if (!pistonDone) {
            System.out.println("PISTON NOT DONE, CHECKING");
            if (startTime > 0 && System.currentTimeMillis() - startTime > 500) {
                m_arm.setElbowSetpoint(ArmConstants.kElbowHighExtendRotations);
                pistonDone = true;
            }
        } else {
            System.out.println("PISTON DONE, EXTENDING ELBOW");
            m_arm.setElbowSetpoint(ArmConstants.kElbowHighExtendRotations);
        }
        return false;
    }
}
