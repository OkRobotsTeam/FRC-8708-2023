package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveFor extends CommandBase{
    
    private final double m_distance;
    private final double m_speed;
    private final Drivetrain m_drive;
    private final boolean m_fast;
    private final double cmPerRot;

    public DriveFor(double distance_cm, double speed, Drivetrain drive){
        m_fast = false;
        m_speed = speed;
        m_distance = distance_cm;
        m_drive = drive;
        cmPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        addRequirements(drive);
    }
    
    public DriveFor(double distance_cm, double speed, Drivetrain drive, boolean fast) {
        m_distance = distance_cm;
        m_speed = speed;
        m_drive = drive;
        m_fast = fast;
        if (fast) {
            cmPerRot = DriveConstants.kFastRevPerRot * DriveConstants.kWheelCircumference;
        } else {
            cmPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        }
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.resetEncoders();
        m_drive.tankDrive(0,0,m_fast);
    }

    @Override
    public void execute() {
        m_drive.tankDrive(m_speed, m_speed, m_fast);
    }

    @Override
    public boolean isFinished() {
        double avgDistance = m_drive.getAvgEncoder() * cmPerRot;
        return (avgDistance >= m_distance);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0,0,false);
    }
}
