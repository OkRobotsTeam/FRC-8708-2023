package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutonSimple extends SequentialCommandGroup {

    public AutonSimple(
        Drivetrain drive,
        Arm arm,
        Intake intake
    ) {
        addCommands(
            new DriveFor(100, 1, drive),
            new TurnFor(180, 0.5, drive)
        );

    }
}
