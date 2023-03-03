package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class Fetch extends CommandBase { 
    DriveSubsystemMax m_drive;
    PickupSubsystem m_pickup;
    TransferSubsystem m_transfer;
    double m_timeout;
    Timer timer = new Timer();

    public Fetch(DriveSubsystemMax drive, PickupSubsystem pickup, TransferSubsystem transfer) {
        m_drive = drive;
        m_pickup = pickup;
        m_transfer = transfer; 
        timer.reset();
        timer.start();
        addRequirements(m_drive);
        addRequirements(m_pickup);
        addRequirements(m_transfer);
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
