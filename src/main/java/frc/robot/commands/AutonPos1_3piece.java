package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.experimental.DriveForTrap;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;

// Red Cable Ramp

public class AutonPos1_3piece extends SequentialCommandGroup {
    public AutonPos1_3piece(
            Drivetrain drive,
            Arm arm,
            Intake intake) {
        //*        
        addCommands(
            new InstantCommand(() -> drive.gyro.reset(), drive),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTrap(5, 1, drive, true),
            new TurnTo(10, 0.6, drive),
            new DriveForTrap(36, 1, drive, true),
            new TurnTo(0, 0.6, drive),
            new DriveForTrap(90, 1, drive, false),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveForTrap(30, 1, drive, true),
            new WaitCommand(0.5), // Added this line
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTrap(-120, 1, drive, true),
            new WaitCommand(0.75),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTrap(78, 1, drive, false),
            new TurnTo(-28, 0.6, drive),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new InstantCommand(intake::intakeIn, intake),
            new DriveForTrap(60, 1, drive, false),
            new WaitCommand(0.5), // Added this line
            new InstantCommand(intake::intakeStop, intake),
            new DriveForTrap(-82, 1, drive, false),
            new TurnTo(-10, 0.6, drive),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTrap(-82, 1, drive, false),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> drive.setBrakeMode(false), drive)
        );
        //*/
        /*
        addCommands(
            new InstantCommand(() -> drive.gyro.reset(), drive),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTrap(0, 5, 0.9, drive, true,20,1),
            // new TurnTo(-10, 0.6, drive),
            new DriveForTrap(10, 5, 0.9, drive, true,1, 1),
            new DriveForTrap(10, 26, 1, drive, true, 1, 1),
            new DriveForTrap(10, 5, 0.9, drive, true,1,1),
            // new TurnTo(0, 0.6, drive),
            new DriveForTrap(0, 5, 0.9, drive, false,1,1),
            new DriveForTrap(0, 85, 1, drive, false,1,1),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveForTrap(0, 27, 1, drive, false,1,20),
            new WaitCommand(0.5), // Added this line
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTrap(0, -120, 1, drive, false),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTrap(0, 78, 1, drive, false,20,1),
            // new TurnTo(28, 0.6, drive),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new InstantCommand(intake::intakeIn, intake),
            new DriveForTrap(-28, 5, 0.9, drive, false,1,1),
            new DriveForTrap(-28, 55, 1, drive, false,1, 1),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new DriveForTrap(-28, -77, 1, drive, false,20,1),
            new DriveForTrap(-28, -5, 0.9, drive, false,1,1),
            // new TurnTo(10, 0.6, drive),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTrap(-10, -5, 0.9, drive, false,1,1),
            new DriveForTrap(-10, -77, 1, drive, false,1,20),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> drive.setBrakeMode(false), drive)
        );
        */
    }
}
