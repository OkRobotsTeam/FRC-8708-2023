package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutonTest extends SequentialCommandGroup {
    public AutonTest(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        addCommands(
            new DriveForTick(0,12,0.5,drive,true,20,20)
  
        );
    }
}
