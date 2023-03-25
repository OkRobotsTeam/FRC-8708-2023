package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonNothing extends SequentialCommandGroup {
    public AutonNothing() {
        addCommands(new WaitCommand(15));
    }
}