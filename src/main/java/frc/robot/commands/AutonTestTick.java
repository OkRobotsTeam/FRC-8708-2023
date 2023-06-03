package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// 18.85 in/rot

public class AutonTestTick extends SequentialCommandGroup {
    public AutonTestTick(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        addCommands(
            
            new InstantCommand(() -> drive.gyro.reset(), drive),
            new DriveForTick(0, 36, 1, drive, false ,20, 20),
            new InstantCommand(()->drive.setBrakeMode(false),drive)
  
        );
    }
}
