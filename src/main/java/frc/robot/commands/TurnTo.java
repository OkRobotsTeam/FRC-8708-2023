package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DriveConstants;

public class TurnTo extends CommandBase {

    private final double m_targetHeading;
    private final double m_speed;
    private final Drivetrain m_drive;
    private final boolean m_fast;
    private double delta_heading;
    private double startTime = -1;


    public TurnTo(double heading, double speed, Drivetrain drive) {
        m_fast = false;
        m_speed = speed;
        m_targetHeading = heading;
        m_drive = drive;
        addRequirements(drive);
    }

    public TurnTo(double heading, double speed, Drivetrain drive, boolean fast) {
        m_targetHeading = heading % 360;
        m_speed = speed;
        m_drive = drive;
        m_fast = fast;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        m_drive.tankDriveRaw(0, 0, m_fast);
        m_drive.setBrakeMode(true);
        startTime = System.currentTimeMillis();
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
            m_drive.tankDriveRaw(Math.max(-m_speed, -m_speed * (Math.abs(delta_heading) * DriveConstants.kTurnAggression)), Math.min(m_speed, m_speed * (Math.abs(delta_heading) * DriveConstants.kTurnAggression)), m_fast);
        } else {
            delta_heading = right_turn_difference;
            m_drive.tankDriveRaw(Math.min(m_speed, m_speed * (Math.abs(delta_heading) * DriveConstants.kTurnAggression)), Math.max(-m_speed, -m_speed * (Math.abs(delta_heading) * DriveConstants.kTurnAggression)), m_fast);
        }
    }

    @Override
    public boolean isFinished() {
        if (startTime > 0 && System.currentTimeMillis() - startTime > 3000) {
            System.out.println("TurnTo terminated by timeout");
            return true;
        }
        System.out.println("TO GO: " + delta_heading);
        return Math.abs(delta_heading) <= Math.abs(DriveConstants.kAllowableHeadingOffset);
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0, 0, false, false);
        System.out.println("DONE");
    }
}
