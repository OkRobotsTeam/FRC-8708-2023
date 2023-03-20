package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;

public class MoveToMid extends CommandBase {

    private Arm m_arm;

    public MoveToMid(Arm arm) {
        m_arm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        boolean elbowExtended = m_arm.getElbowExtended();
        if (elbowExtended) {
            m_arm.setElevatorExtended(elbowExtended);
        }
        return elbowExtended;
    }

    @Override
    public void end(boolean interrupted) {
        m_arm.setElevatorExtended(true);
    }
}
