package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.experimental.DriveForTrap;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// 18.85 in/rot

public class AutonTest extends SequentialCommandGroup {
    public AutonTest(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        addCommands(
            
            new InstantCommand(() -> drive.gyro.reset(), drive),
            // new TurnTo(90, 0.6, drive),
            // new TurnTo(60, 0.6, drive),
            // new WaitCommand(2),
            // new TurnTo(70, 0.6, drive),
            // new DriveForTick(0, 20, 0.7, drive, false, 20, 1),
            // new DriveForTick(90, 40, 0.7, drive, false, 1, 20),

            new DriveForTrap(20, 1, drive, true),
            // new TurnTo(90,0.7,drive),
            // new DriveForTrap(20,1,drive,true),
            // new TurnTo(180,0.7,drive),
            // new DriveForTrap(10, 1, drive, false),
            // new DriveForTrap(10,1,drive,true),
            // new TurnTo(270,0.7,drive),
            // new DriveForTrap(20,1,drive,true),
            // new TurnTo(360,0.7,drive),

            new WaitCommand(2),
            new InstantCommand(()->drive.setBrakeMode(false),drive)
  
        );
    }
}
