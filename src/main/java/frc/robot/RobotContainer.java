// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonSimple;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.vision.VisionThread;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elbow;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Arm m_arm = new Arm();
  private final Elbow m_elbow = new Elbow(m_arm);
  private final Intake m_intake = new Intake();
  private final Lights m_lights = new Lights();
  private final CommandJoystick m_driverLeftJoystick = new CommandJoystick(OperatorConstants.kDriverLeftJoystickPort);
  private final CommandJoystick m_driverRightJoystick = new CommandJoystick(OperatorConstants.kDriverRightJoystickPort);

  private final CommandXboxController m_manipulator = new CommandXboxController(
      OperatorConstants.kManipulatorControllerPort);

  private VisionThread m_visionThread;

  public boolean m_webcamPresent;

   // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    m_webcamPresent = (CameraServerJNI.enumerateUsbCameras().length > 0);

    if (m_webcamPresent) {
      System.out.println("Vision active");
      m_visionThread = new VisionThread();
      m_visionThread.setDaemon(true);
      m_visionThread.start();
      m_visionThread.setPriority(3);
    } else {
      System.out.println("No webcam found, vision inactive");
    }

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // A = intake in
    // RT = intake out fast
    // LT = elbow out/in (hold)
    // LB/RB = adjust elbow
    // Start/Back = colors purple start yellow back

    m_drivetrain.setDefaultCommand(getTankDriveCommand());

    m_manipulator.a().onTrue(
        new InstantCommand(
            () -> m_intake.intakeIn(0.5), m_intake));
    m_manipulator.rightTrigger().onTrue(
        new InstantCommand(
            () -> m_intake.intakeOut(0.5), m_intake));
    m_manipulator.a().onFalse(
        new InstantCommand(
            m_intake::intakeStop, m_intake));
    m_manipulator.rightTrigger().onFalse(
        new InstantCommand(
            m_intake::intakeStop, m_intake));
    m_manipulator.povUp().onTrue(
        new InstantCommand(
            () -> m_arm.setPistonRaised(true), m_arm));
    m_manipulator.povDown().onTrue(
        new InstantCommand(
            () -> m_arm.setPistonRaised(false), m_arm));
    m_manipulator.povLeft().onTrue(
        new InstantCommand(
            () -> m_arm.setElevatorExtended(false), m_arm));
    m_manipulator.povRight().onTrue(
        new InstantCommand(
            () -> m_arm.setElevatorExtended(true), m_arm));
    m_manipulator.leftTrigger().onTrue(
        new InstantCommand(
            () -> m_elbow.setElbowExtended(true), m_arm));
    m_manipulator.leftTrigger().onFalse(
        new InstantCommand(
            () -> m_elbow.setElbowExtended(false), m_arm));

    m_manipulator.start().onTrue(
        new InstantCommand(
            m_lights::setViolet, m_lights).andThen(
                new WaitCommand(OperatorConstants.kLightsTimeoutSeconds),
                new InstantCommand(m_lights::setChaser, m_lights)));
    
    m_manipulator.back().onTrue(
        new InstantCommand(
            m_lights::setYellow, m_lights).andThen(
                new WaitCommand(OperatorConstants.kLightsTimeoutSeconds),
                new InstantCommand(m_lights::setChaser, m_lights)));
    
    m_manipulator.leftBumper().onTrue(
        new InstantCommand(
        () -> m_elbow.tuneTarget(1.0), m_elbow));
    
    m_manipulator.rightBumper().onTrue(
        new InstantCommand(
        () -> m_elbow.tuneTarget(-1.0), m_elbow));
  }

  public void teleopInit() {
     m_arm.teleopInit();
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutonSimple(m_drivetrain, m_arm, m_elbow, m_intake);
  }

  public Command getTankDriveCommand() {
    return new RunCommand(
        () -> m_drivetrain.tankDrive(
            m_driverLeftJoystick.getY(),
            m_driverRightJoystick.getY(),
            m_driverRightJoystick.trigger().getAsBoolean(),
            m_driverLeftJoystick.trigger().getAsBoolean()),
        m_drivetrain);
  }
}