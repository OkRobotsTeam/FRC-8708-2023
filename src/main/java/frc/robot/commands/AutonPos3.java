package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// blue cable ramp

public class AutonPos3 extends SequentialCommandGroup {
    public AutonPos3(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        addCommands(
            new InstantCommand(() -> drive.gyro.reset(), drive),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveFor(0, 5, 0.7, drive, true),
            new TurnTo(-10, 0.4, drive),
            new DriveFor(-10, 36, 0.7, drive, true),
            new TurnTo(0, 0.4, drive),
            new DriveFor(0, 90, 0.7, drive, false),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveFor(0, 40, 0.7, drive, false),
            new WaitCommand(0.5), // Added this line
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveFor(0, -125, 0.7, drive, false),
            new WaitCommand(1),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> drive.setBrakeMode(false), drive),
            new WaitCommand(1),
            new DriveFor(0, -18, 0.2, drive, false)
        );
    }
}
