package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

public class AutonPos1 extends SequentialCommandGroup {

    public AutonPos1(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
                addCommands(
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveFor(0, 5, 0.7, drive, true),
            new TurnTo(5, 0.4, drive),
            new DriveFor(5, 36, 0.7, drive, true),
            new TurnTo(0, 0.4, drive),
            new DriveFor(0, 150, 0.7, drive, false),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveFor(0, 20, 0.7, drive, false),
            new DriveFor(0, -10, 0.7, drive, false),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new DriveFor(0, -180, 0.7, drive, false),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveFor(0, -30, 0.7, drive, true),
            new WaitCommand(0.75),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new WaitCommand(1),
            new InstantCommand(() -> drive.setBrakeMode(false), drive)
        );


        /*
        addCommands(
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.5),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveFor(34, 1, drive),
            // TODO: V Replace with TurnTo V
            // new TurnFor(6, 0.5, drive),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveFor(100, 1, drive, false),
            new InstantCommand(intake::intakeIn, intake),
            new DriveFor(30, 1, drive, false),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new DriveFor(-130, 1, drive, false),
            // new TurnFor(-8, 0.5, drive),
            // new DriveFor(-34, 0.5, drive),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.5),
            new InstantCommand(intake::intakeStop, intake),
            new DriveFor(100, 1, drive, false),
            // TODO: V Replace with TurnTo V
            // new TurnFor(90, 1, drive, false),
            new DriveFor(40, 1, drive, false),
            //new TurnFor(90, 0.5, drive, false),
            //new DriveFor(70, 0.7, drive, false),

            new WaitCommand(1),
            new InstantCommand(() -> drive.setBrakeMode(false), drive)
        );
        */

    }
}