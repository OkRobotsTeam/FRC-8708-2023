package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSimple;
import frc.robot.subsystems.TransferSubsystem;

public class Shoot extends CommandBase { 
    ShooterSimple m_shooter;
    TransferSubsystem m_transfer;
    Timer feedTimer = new Timer();
    Timer timeoutTimer = new Timer();
    boolean spinningUp = false;
    public Shoot(ShooterSimple shooter, TransferSubsystem transfer) {
        addRequirements(shooter, transfer);
        m_transfer = transfer;
        m_shooter = shooter;
    }

    @Override    
    public void initialize() {
        spinningUp = true;
        m_shooter.startFlywheel();
        timeoutTimer.reset();
        timeoutTimer.start();
    }

    @Override    
    public void execute() {
        if (spinningUp) {
            if (m_shooter.atSetpoint()) {
                m_shooter.runFeeder();
                m_transfer.run();
                feedTimer.reset();
                feedTimer.start();
                spinningUp=false;
            }
        }
    }

    @Override    
    public boolean isFinished() {
        if (spinningUp) {
            return timeoutTimer.hasElapsed(Constants.ShooterConstants.kShootTimeoutSeconds);
        } else {
            return feedTimer.hasElapsed(Constants.ShooterConstants.kShootTimeSeconds);
        }
    }

    @Override    
    public void end(boolean interrupted) {
        m_transfer.stop();
        m_shooter.disable();
    }
    
}
