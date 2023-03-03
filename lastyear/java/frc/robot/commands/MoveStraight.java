package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystemMax;

public class MoveStraight extends CommandBase { 
    DriveSubsystemMax m_drive;
    Double m_speed;
    Double m_stopAfterSeconds=0.0;
    Timer timer = new Timer();

    public MoveStraight(Double speed, Double stopAfterSeconds, DriveSubsystemMax drive) {
        m_drive = drive;
        m_speed = speed;
        m_stopAfterSeconds = stopAfterSeconds;
        addRequirements(m_drive);
    }

    @Override    
    public void initialize() {
        m_drive.forward();
        timer.reset();
        timer.start();
    }

    @Override    
    public void execute() {
        m_drive.moveStraight(m_speed);
    }

    @Override    
    public boolean isFinished() {
        System.out.println("Move timeout:" + timer.get() + "<" + m_stopAfterSeconds);
        return timer.hasElapsed(m_stopAfterSeconds);
    }

    @Override    
    public void end(boolean interrupted) {
        m_drive.stop();
    }
    
}
