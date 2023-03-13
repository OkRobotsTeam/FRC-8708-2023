package frc.robot.commands;

import frc.robot.utils.Vector;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveToCoords extends CommandBase {

    private final Drivetrain m_drive;
    private final Vector m_currpos;
    private final Vector m_newpos;

    public DriveToCoords(Vector newpos, Drivetrain drive) {
        m_drive = drive;
        m_newpos = newpos;
        addRequirements(drive);

        double[] limelightPosData = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[]{-100.0,-100.0});
        m_currpos = new Vector(limelightPosData[0],limelightPosData[1]);

    }


}