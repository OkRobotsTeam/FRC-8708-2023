
package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.*;

public class MoreBallsTest extends SequentialCommandGroup {

  public MoreBallsTest(DriveSubsystemMax m_robotDrive, ShooterSimple m_shooter, TransferSubsystem m_transfer, PickupSubsystem m_pickup) {
    
    addRequirements(m_robotDrive, m_shooter, m_transfer, m_pickup);
     /*addCommands(new WaitCommand(1));
     addCommands(new PrintCommand("After 1"));
     addCommands(new WaitCommand(2));
     addCommands(new PrintCommand("Moving"));


     addCommands(new PrintCommand("Stopped"));
     */
    addCommands(new MoveStraight(0.5, 1.6, m_robotDrive));
    addCommands(new MoveStraight(0.5, 1.6, m_robotDrive));


     //addCommands(new Shoot(m_shooter,m_transfer) );
     addCommands(new PrintCommand("After Shoot"));

  }

}
