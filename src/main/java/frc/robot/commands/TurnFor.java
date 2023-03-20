package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.Constants.DriveConstants;

public class TurnFor extends CommandBase {

    private final double m_degrees;
    private final double m_speed;
    private final Drivetrain m_drive;
    private final boolean m_fast;
    private final double cmPerRot;

    private final PIDController leftPID = new PIDController(0.1, 0, 0);
    private final PIDController rightPID = new PIDController(0.1, 0, 0);

    private double leftDesiredSpeed;
    private double rightDesiredSpeed;

    public TurnFor(double degrees, double speed, Drivetrain drive) {
        m_fast = false;
        m_speed = speed;
        m_degrees = degrees;
        m_drive = drive;
        cmPerRot = DriveConstants.kSlowRevPerRot * DriveConstants.kWheelCircumference;
        addRequirements(drive);
    }

    public TurnFor(double degrees, double speed, Drivetrain drive, boolean fast) {
        m_degrees = degrees;
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
        leftStartPos = m_drive.getLeftEncoder();
        rightStartPos = m_drive.getRightEncoder();
        m_drive.tankDrive(0, 0, m_fast, false);
        m_drive.setBrakeMode(true);
    }

    @Override
    public void execute() {
        if (m_degrees > 0) {
            m_drive.tankDrive(m_speed, -m_speed, m_fast, false);
        } else {
            m_drive.tankDriveRaw(m_speed, -m_speed, m_fast);
        }
    }

    @Override
    public boolean isFinished() {
        double avgWheelRev = Math.abs(((m_drive.getLeftEncoder() - leftStartPos) - (m_drive.getRightEncoder() - rightStartPos)) / 2);
        double degreesTurned = avgWheelRev * cmPerRot / DriveConstants.kTurnCircumference * 360 / DriveConstants.kTurnError;
        System.out.println("GONE: "+degreesTurned);
        return (degreesTurned >= Math.abs(m_degrees));
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.tankDrive(0, 0, false, false);
        System.out.println("DONE");
    }
}
