package frc.robot.commands;


import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.HangerSubsystem;

public class AutoRetractHanger extends CommandBase { 
    HangerSubsystem m_hanger;
    double m_timeout;
    Timer timer = new Timer();

    public AutoRetractHanger(HangerSubsystem hanger) {
        m_hanger = hanger;
        addRequirements(hanger);
    }

    @Override    
    public void initialize() {
    }

    @Override    
    public void execute() {
        
        if ((DriverStation.getMatchTime()<2) && (DriverStation.getMatchTime() > 0)) {
            m_hanger.retract();
        }
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
    }
    
}
