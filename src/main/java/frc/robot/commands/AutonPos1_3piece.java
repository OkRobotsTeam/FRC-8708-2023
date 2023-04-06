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
        //*        
        addCommands(
            new InstantCommand(() -> drive.gyro.reset(), drive),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTick(0, 5, 1, drive, true,20,20),
            new TurnTo(10, 0.6, drive),
            new DriveForTick(10, 36, 1, drive, true,20,20),
            new TurnTo(0, 0.6, drive),
            new DriveForTick(0, 90, 1, drive, false,20,1),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveForTick(0, 27, 1, drive, false,1,20),
            new WaitCommand(0.5), // Added this line
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTick(0, -120, 1, drive, false,20,20),
            new WaitCommand(0.75),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTick(0, 78, 1, drive, false,20,20),
            new TurnTo(-28, 0.6, drive),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new InstantCommand(intake::intakeIn, intake),
            new DriveForTick(-28, 60, 1, drive, false,20,20),
            new WaitCommand(0.5), // Added this line
            new InstantCommand(intake::intakeStop, intake),
            new DriveForTick(-28, -82, 1, drive, false,20,20),
            new TurnTo(-10, 0.6, drive),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTick(-10, -82, 1, drive, false,20,20),
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
            new DriveForTick(0, 5, 0.9, drive, true,20,1),
            // new TurnTo(-10, 0.6, drive),
            new DriveForTick(10, 5, 0.9, drive, true,1, 1),
            new DriveForTick(10, 26, 1, drive, true, 1, 1),
            new DriveForTick(10, 5, 0.9, drive, true,1,1),
            // new TurnTo(0, 0.6, drive),
            new DriveForTick(0, 5, 0.9, drive, false,1,1),
            new DriveForTick(0, 85, 1, drive, false,1,1),
            new InstantCommand(intake::intakeIn, intake),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new DriveForTick(0, 27, 1, drive, false,1,20),
            new WaitCommand(0.5), // Added this line
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTick(0, -120, 1, drive, false,20,20),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setPistonRaised(false), arm),
            new DriveForTick(0, 78, 1, drive, false,20,1),
            // new TurnTo(28, 0.6, drive),
            new InstantCommand(() -> arm.setElbowExtended(true), arm),
            new InstantCommand(intake::intakeIn, intake),
            new DriveForTick(-28, 5, 0.9, drive, false,1,1),
            new DriveForTick(-28, 55, 1, drive, false,1, 1),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> arm.setElbowExtended(false), arm),
            new DriveForTick(-28, -77, 1, drive, false,20,1),
            new DriveForTick(-28, -5, 0.9, drive, false,1,1),
            // new TurnTo(10, 0.6, drive),
            new InstantCommand(() -> arm.setPistonRaised(true), arm),
            new DriveForTick(-10, -5, 0.9, drive, false,1,1),
            new DriveForTick(-10, -77, 1, drive, false,1,20),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeOut, intake),
            new WaitCommand(0.2),
            new InstantCommand(intake::intakeStop, intake),
            new InstantCommand(() -> drive.setBrakeMode(false), drive)
        );
        */
    }
}
