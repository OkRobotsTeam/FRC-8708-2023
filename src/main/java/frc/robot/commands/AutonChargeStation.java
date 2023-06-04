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
            new WaitCommand(1),
            new DriveFor(0, 91, 0.35, drive, true),
            new InstantCommand(() -> drive.setBrakeMode(true), drive),
            new WaitCommand(100)
        );
    }
}
