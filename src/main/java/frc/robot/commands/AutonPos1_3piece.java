package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// Red Cable Ramp

public class AutonPos1_3piece extends SequentialCommandGroup {
    public AutonPos1_3piece(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        addCommands(
            // new InstantCommand(() -> drive.gyro.reset(), drive),
            // new InstantCommand(intake::intakeOut, intake),
            // new WaitCommand(0.2),
            // new InstantCommand(intake::intakeStop, intake),
            // new InstantCommand(() -> arm.setPistonRaised(false), arm),
            // new DriveFor(0, 5, 0.7, drive, true),
            // new TurnTo(10, 0.4, drive),
            // new DriveFor(10, 36, 0.7, drive, true),
            // new TurnTo(0, 0.4, drive),
            // new DriveFor(0, 90, 0.7, drive, false),
            // new InstantCommand(intake::intakeIn, intake),
            // new InstantCommand(() -> arm.setElbowExtended(true), arm),
            // new DriveFor(0, 40, 0.7, drive, false),
            // new WaitCommand(0.5), // Added this line
            // new InstantCommand(intake::intakeStop, intake),
            // new InstantCommand(() -> arm.setElbowExtended(false), arm),
            // new InstantCommand(() -> arm.setPistonRaised(true), arm),
            // new DriveFor(0, -140, 0.7, drive, false),
            // new WaitCommand(0.75),
            // new InstantCommand(intake::intakeOut, intake),
            // new WaitCommand(0.2),
            // new InstantCommand(intake::intakeStop, intake),
            // new InstantCommand(() -> drive.setBrakeMode(false), drive),
            // new InstantCommand(() -> arm.setPistonRaised(false), arm),
            // new WaitCommand(1),
            // new DriveFor(0, 96, 0.7, drive, false),
            // new TurnTo(-28, 0.7, drive),
            // new InstantCommand(() -> arm.setElbowExtended(true), arm),
            // new InstantCommand(intake::intakeIn, intake),
            // new DriveFor(-28, 82, 0.7, drive, false),
            // new InstantCommand(intake::intakeStop, intake),
            // new InstantCommand(() -> arm.setElbowExtended(false), arm),
            // new DriveFor(-38, -94, 0.7, drive, false),
            // new TurnTo(10, 0.7, drive),
            // new InstantCommand(() -> arm.setPistonRaised(true), arm),
            // new DriveFor(10, -60, 0.7, drive, false),
            // new TurnTo(30, 0.7, drive),
            // new DriveFor(30, -10, 0.7, drive, false),
            // new InstantCommand(intake::intakeOut, intake),
            // new WaitCommand(0.2),
            // new InstantCommand(intake::intakeStop, intake)

            new InstantCommand(() -> drive.gyro.reset(), drive),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTick(0, 5, 1, drive, false, 20, 20),
            new TurnTo(10, 1, drive),
            new DriveForTick(10, 36, 1, drive, false ,20, 20),
            new TurnTo(0, 0.7, drive),
            new DriveForTick(0, 90, 1, drive, false, 20, 0),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveForTick(0, 40, 1, drive, false, 0, 20),
            new WaitCommand(0.5),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTick(0, -140, 1, drive, false, 20, 20),
            new WaitCommand(0.75),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new WaitCommand(1),
            new DriveForTick(0, 96, 1, drive, false, 20, 20),
            new TurnTo(-28, 0.7, drive),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new InstantCommand(intake::intakeIn, intake),
            new DriveForTick(-28, 82, 1, drive, false, 20, 20),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new DriveForTick(-28, -94, 1, drive, false, 20, 20),
            new TurnTo(10, 0.7, drive),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTick(10, -60, 1, drive, false, 20, 20),
            new TurnTo(30, 0.7, drive),
            new DriveForTick(30, -10, 1, drive, false, 20, 20),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake)
        );
    }
}
