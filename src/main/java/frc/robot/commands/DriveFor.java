package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveFor extends CommandBase {

    private final double m_distance;
    private final double m_speed;
    private final Drivetrain m_drive;
    private final double cmPerRot;
    private final boolean m_brake;

    private double start_pos;

    public DriveFor(double distance_in, double unsigned_speed, Drivetrain drive) {
        if (distance_in < 0) {
            m_speed = -unsigned_speed;
        } else {
            m_speed = unsigned_speed;
        }
        m_distance = distance_in;
        m_drive = drive;
        cmPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        m_brake = true;
        addRequirements(drive);
    }

    public DriveFor(double distance_in, double unsigned_speed, Drivetrain drive, boolean brake) {
        m_distance = distance_in;
        if (distance_in < 0) {
            m_speed = -unsigned_speed;
        } else {
            m_speed = unsigned_speed;
        }
        m_drive = drive;
        m_brake = brake;
        cmPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        start_pos = m_drive.getAvgEncoder();
        m_drive.tankDriveRaw(0, 0, false);
        System.out.println("DISTANCE TO GO: "+m_distance);
        m_drive.setBrakeMode(m_brake);
        m_drive.setRampRate(0.5);
    }

    @Override
    public void execute() {
        m_drive.tankDriveRaw(-m_speed, -m_speed, false);
        
    }

    @Override
    public boolean isFinished() {
        double avgDistance = Math.abs((m_drive.getAvgEncoder()-start_pos) * cmPerRot);
        System.out.println("GONE: "+avgDistance);
        return (avgDistance >= Math.abs(m_distance));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setRampRate(0);
        m_drive.tankDriveRaw(0, 0, false);
        System.out.println("Done");
    }
}
