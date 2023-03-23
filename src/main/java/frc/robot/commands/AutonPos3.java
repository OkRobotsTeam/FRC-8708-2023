package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutonPos3 extends SequentialCommandGroup {
    public AutonPos3(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        addCommands(

            //...
            new WaitCommand(1),
            new InstantCommand(() -> drive.setBrakeMode(false), drive)
        );
    }
}
