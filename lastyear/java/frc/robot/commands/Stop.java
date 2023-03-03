package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystemMax;

public class Stop extends CommandBase { 
    DriveSubsystemMax m_drive;
    double m_timeout;
    Timer timer = new Timer();

    public Stop(DriveSubsystemMax drive, double timeout) {
        m_drive = drive;
        m_timeout = timeout;
        timer.reset();
        timer.start();
        addRequirements(m_drive);
    }

    @Override    
    public void initialize() {
        m_drive.stop();
    }

    @Override    
    public void execute() {
        m_drive.stop();
    }

    @Override    
    public boolean isFinished() {
        if (m_timeout > 0) {
            return timer.hasElapsed(m_timeout);
        } else {
            return false;
        }
    }

    @Override    
    public void end(boolean interrupted) {
        m_drive.stop();
    }
    
}
