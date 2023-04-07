package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.experimental.DriveForTrap;
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
            // new TurnTo(90, 0.6, drive),
            // new TurnTo(60, 0.6, drive),
            // new WaitCommand(2),
            // new TurnTo(70, 0.6, drive),
            new DriveForTick(0, 40, 0.5, drive, false, 20, 0),
            new DriveForTick(90, 90, 0.5, drive, false, 0, 20),

            new DriveForTick(90, -90, 0.5, drive, false, 20, 0),
            //new DriveForTick(0, 40, 0.7, drive, false, 0, 20),

            new InstantCommand(()->drive.setBrakeMode(false),drive)
  
        );
    }
}
