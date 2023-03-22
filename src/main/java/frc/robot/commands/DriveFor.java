package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class DriveFor extends CommandBase {

    private final double m_distance;
    private final double m_targetHeading;
    private final double m_speed;
    private final Drivetrain m_drive;
    private final double cmPerRot;
    private final boolean m_brake;
    private double delta_heading;

    private double start_pos;

    public DriveFor(double heading, double distance_in, double unsigned_speed, Drivetrain drive, boolean brake) {
        m_targetHeading = heading;
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
        System.out.println("DISTANCE TO GO: " + m_distance);
        m_drive.setBrakeMode(m_brake);
        m_drive.setRampRate(0.5);
    }

    @Override
    public void execute() {
        double current_heading = m_drive.gyro.getAngle() % 360;
        double left_turn_difference = (current_heading - m_targetHeading);
        double right_turn_difference = (m_targetHeading - current_heading);
        if (left_turn_difference < 0) {
            left_turn_difference += 360;
        }
        if (right_turn_difference < 0) {
            right_turn_difference += 360;
        }
        if (Math.abs(left_turn_difference) < Math.abs(right_turn_difference)) {
            delta_heading = left_turn_difference;
            m_drive.tankDriveRaw((delta_heading * -DriveConstants.kCorrectionAggression) - m_speed, (delta_heading * DriveConstants.kCorrectionAggression) - m_speed, false);
        } else {
            delta_heading = right_turn_difference;
            m_drive.tankDriveRaw((delta_heading * DriveConstants.kCorrectionAggression) - m_speed, (delta_heading * -DriveConstants.kCorrectionAggression) - m_speed, false);
        }
        
    }

    @Override
    public boolean isFinished() {
        double avgDistance = Math.abs((m_drive.getAvgEncoder()-start_pos) * cmPerRot);
        System.out.println("GONE: " + avgDistance);
        System.out.println("DEGREES OFF COURSE: " + Math.abs(delta_heading));
        return (avgDistance >= Math.abs(m_distance));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.setRampRate(0);
        m_drive.tankDriveRaw(0, 0, false);
        System.out.println("Done");
    }
}
