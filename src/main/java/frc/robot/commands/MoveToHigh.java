package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MoveToHigh extends CommandBase {

    private Arm m_arm;
    private double startTime = -1;

    public MoveToHigh(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        m_arm.setElbowSetpoint(ArmConstants.kElbowHighExtendRotations);
        startTime = System.currentTimeMillis();
    }

    @Override
    public boolean isFinished() {
        if (startTime > 0 && System.currentTimeMillis() - startTime > 3000) {
            System.out.println("MoveToHigh terminated by timeout");
            return true;
        }
        double elbowPosition = m_arm.getElbowPosition();
        if (Math.abs(elbowPosition - ArmConstants.kElbowHighExtendRotations) < 3) {
            m_arm.setElevatorExtended(true);
            return true;
        }
        return false;
    }
}
