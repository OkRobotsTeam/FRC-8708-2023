
package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystemMax;

public class SequentialTest extends SequentialCommandGroup {

  public SequentialTest(DriveSubsystemMax drive) {
     addCommands(new WaitCommand(1),
      new PrintCommand("After 1"),
      new WaitCommand(2),
      new PrintCommand("After 2"));
     
  }




}
