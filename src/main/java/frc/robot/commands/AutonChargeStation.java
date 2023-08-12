package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// blue cable ramp

public class AutonChargeStation extends SequentialCommandGroup {
    public AutonChargeStation(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        addCommands(
            new InstantCommand(() -> drive.gyro.reset(), drive),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new WaitCommand(1.5),
            // new DriveFor(0, 87, 0.35, drive, true),
            new DriveFor(0, 85, 0.35, drive, true),
            new InstantCommand(() -> drive.setBrakeMode(true), drive),
            new WaitCommand(8.5),
            // new DriveFor(0, 5, 0.3, drive, true),
            new DriveFor(0, 4.5, 0.15, drive, true),
            new WaitCommand(100)
        );
    }
}
