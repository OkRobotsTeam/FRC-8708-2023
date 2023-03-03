
package frc.robot.commands;


import javax.management.NotificationBroadcasterSupport;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemMax;
import frc.robot.subsystems.PickupSubsystem;
import frc.robot.subsystems.ShooterSimple;
import frc.robot.subsystems.TransferSubsystem;

public class TwoBallAuto extends SequentialCommandGroup {

  public TwoBallAuto(DriveSubsystemMax m_robotDrive, ShooterSimple m_shooter, 
  TransferSubsystem m_transfer, PickupSubsystem m_pickup ) {
      addCommands(new InstantCommand(m_shooter::startFlywheel,m_shooter));
      addCommands(new InstantCommand(m_pickup::armDown, m_pickup));
      addCommands(new InstantCommand(m_pickup::run, m_pickup));
      addCommands(new InstantCommand(m_transfer::run, m_transfer));
      addCommands(new InstantCommand(m_shooter::runFeederBackwards,m_shooter));
      addCommands(new MoveStraight(-0.65, 1.6, m_robotDrive));
      addCommands(new TurnToTarget(m_robotDrive));
      addCommands(new WaitCommand(2));

      addCommands(new InstantCommand(m_pickup::stop, m_pickup));
      addCommands(new InstantCommand(m_shooter::stopFeeder, m_shooter));
      addCommands(new InstantCommand(m_transfer::stop, m_transfer));
      addCommands(new InstantCommand(m_pickup::armUp, m_pickup));

      addCommands(new InstantCommand(m_shooter::twoBallRPM, m_shooter));
      addCommands(new Shoot(m_shooter,m_transfer) );  
  }




}
