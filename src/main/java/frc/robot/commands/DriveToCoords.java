package frc.robot.commands;

import frc.robot.utils.Vector;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class DriveToCoords extends SequentialCommandGroup {


    public DriveToCoords(Vector newpos, double turnspeed, double drivespeed, Drivetrain drive) {
        addRequirements(drive);

        double[] limelightPosData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{-100.0,-100.0});
        Vector currpos = new Vector(limelightPosData[0],limelightPosData[1]);

        Vector movementVector = newpos.minus(currpos);

        addCommands(
            new TurnTo(movementVector.angleDegrees(), turnspeed, drive),
            new DriveFor(movementVector.angleDegrees(), movementVector.magnitude()*39.37, drivespeed, drive, true)
        );

    }


}