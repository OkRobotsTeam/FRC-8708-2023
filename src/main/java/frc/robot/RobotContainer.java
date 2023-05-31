// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutonChargeStation;
import frc.robot.commands.AutonNothing;
import frc.robot.commands.AutonPos1;
import frc.robot.commands.AutonPos1_3piece;
import frc.robot.commands.AutonPos3;
import frc.robot.commands.AutonPos3_3piece;
import frc.robot.commands.AutonTest;
import frc.robot.commands.AutonTestTick;
import frc.robot.commands.MoveToHigh;
import frc.robot.commands.ToggleHigh;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Lights;
import frc.robot.vision.VisionThread1;


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
  private GenericEntry m_objectInIntake = null;
  private final Arm m_arm = new Arm();
  private final Intake m_intake = new Intake(m_arm);
  private final Lights m_lights = new Lights();
  private final CommandJoystick m_driverLeftJoystick = new CommandJoystick(OperatorConstants.kDriverLeftJoystickPort);
  private final CommandJoystick m_driverRightJoystick = new CommandJoystick(OperatorConstants.kDriverRightJoystickPort);

  private final CommandXboxController m_manipulator = new CommandXboxController(
      OperatorConstants.kManipulatorControllerPort);

    private VisionThread1 m_visionThread1;

  public int m_webcamAmount;

  private ShuffleboardTab m_driving_tab = Shuffleboard.getTab("Driving");
  private final SendableChooser<Command> m_autonomous_selecter = new SendableChooser<>();
  private final SendableChooser<Boolean> m_safety_mode = new SendableChooser<>();


   // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    m_webcamAmount = 1;//CameraServerJNI.enumerateUsbCameras().length;

    if (m_webcamAmount > 0) {
      System.out.println("Camera 1 active");
      m_visionThread1 = new VisionThread1();
      m_visionThread1.setDaemon(true);
      m_visionThread1.start();
      m_visionThread1.setPriority(3);
    } else {
      System.out.println("No webcam found, vision inactive");
    }


    m_autonomous_selecter.setDefaultOption("Red cable ramp", new AutonPos1(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Blue cable ramp", new AutonPos3(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Red no cable ramp", new AutonPos3(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Blue no cable ramp", new AutonPos1(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.setDefaultOption("Red cable ramp3", new AutonPos1_3piece(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Blue cable ramp3", new AutonPos3_3piece(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Red no cable ramp3", new AutonPos3_3piece(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Blue no cable ramp3", new AutonPos1_3piece(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Nothing", new AutonNothing());
    m_autonomous_selecter.addOption("TEST DO NOT USE DURING COMP", new AutonTest(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("TEST DO NOT USE DURING COMP TICK", new AutonTestTick(m_drivetrain, m_arm, m_intake));
    m_autonomous_selecter.addOption("Charge Station", new AutonChargeStation(m_drivetrain, m_arm, m_intake));

    m_safety_mode.setDefaultOption("Competition Mode", false);
    m_safety_mode.setDefaultOption("Demonstration Mode", true);

    m_driving_tab.add(m_autonomous_selecter).withPosition(4, 0).withSize(2, 1);
    m_driving_tab.add(m_safety_mode).withPosition(4, 3).withSize(2,1);
    m_objectInIntake = m_driving_tab.add("ObjectInIntake", false).withPosition(4, 1).withSize(2, 2).getEntry();

    Shuffleboard.selectTab("Driving");
    Shuffleboard.update();
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
    m_intake.setDefaultCommand(getIntakeCommand());

    // TEDY'S CONTROLS

    // m_manipulator.a().onTrue(
    //     new InstantCommand(
    //         () -> m_intake.intakeIn(), m_intake));
    // m_manipulator.rightTrigger().onTrue(
    //     new InstantCommand(
    //         () -> m_intake.intakeOut(), m_intake));
    // m_manipulator.a().onFalse(
    //     new InstantCommand(
    //         m_intake::intakeStop, m_intake));
    // m_manipulator.rightTrigger().onFalse(
    //     new InstantCommand(
    //         m_intake::intakeStop, m_intake));
    // m_manipulator.povUp().onTrue(
    //     new InstantCommand(
    //         () -> m_arm.setPistonRaised(true), m_arm));
    // m_manipulator.povDown().onTrue(
    //     new InstantCommand(
    //         () -> m_arm.setPistonRaised(false), m_arm));

    // m_manipulator.leftTrigger().onTrue(
    //     new InstantCommand(
    //         () -> m_arm.setElbowExtended(true), m_arm));
    // m_manipulator.leftTrigger().onFalse(
    //     new InstantCommand(
    //         () -> m_arm.setElbowExtended(false), m_arm));

    // m_manipulator.povLeft().onTrue(
    //     new InstantCommand(
    //         () -> m_arm.setElevatorExtended(false), m_arm));
    // m_manipulator.povRight().onTrue(
    //     new InstantCommand(
    //         () -> m_arm.setElevatorExtended(true), m_arm));

    // m_manipulator.start().onTrue(
    //     new InstantCommand(
    //         m_lights::setViolet, m_lights).andThen(
    //             new WaitCommand(OperatorConstants.kLightsTimeoutSeconds),
    //             new InstantCommand(m_lights::teamChaser,m_lights)));
    
    // m_manipulator.back().onTrue(
    //     new InstantCommand(
    //         m_lights::setYellow, m_lights).andThen(
    //             new WaitCommand(OperatorConstants.kLightsTimeoutSeconds),
    //             new InstantCommand(m_lights::teamChaser,m_lights)));
    
    // m_manipulator.x().onTrue(
    //     new InstantCommand(
    //     () -> m_arm.manualAdjustTarget(1.0), m_arm));
    
    // m_manipulator.b().onTrue(
    //     new InstantCommand(
    //     () -> m_arm.manualAdjustTarget(-1.0), m_arm));
    
    // m_manipulator.leftBumper().onTrue(
    //     new InstantCommand(
    //     () -> m_arm.pickupFromHumanPlayerStation(), m_arm));
    // m_manipulator.leftBumper().onFalse(
    //     new InstantCommand(
    //     () -> m_arm.setElbowExtended(false), m_arm));
    // m_manipulator.rightBumper().onTrue(new MoveToHigh(m_arm));
    // m_manipulator.rightBumper().onFalse(
    //     new InstantCommand(
    //         () -> m_arm.setElbowExtended(false), m_arm).andThen(
    //         () -> m_arm.setElevatorExtended(false), m_arm));

    // HUNTER'S CONTROLS:

    m_manipulator.a().onTrue(
        new InstantCommand(
        () -> m_arm.manualAdjustTarget(1.0), m_arm));

    m_manipulator.b().onTrue(
        new ToggleHigh(m_arm));

    
    m_manipulator.x().onTrue(
        new InstantCommand(
        () -> m_arm.toggleElbowExtended(), m_arm));
    
    
    m_manipulator.y().onTrue(
        new InstantCommand(
        () -> m_arm.manualAdjustTarget(-1.0), m_arm));
    
    m_manipulator.rightBumper().onTrue(
        new InstantCommand(
        () -> m_arm.manualAdjustTarget(3.0), m_arm));
    
    m_manipulator.rightTrigger().onTrue(
        new InstantCommand(
            () -> m_intake.intakeOut(), m_intake));
    m_manipulator.rightTrigger().onFalse(
        new InstantCommand(
            m_intake::intakeStop, m_intake));
    
    
    m_manipulator.povUp().onTrue(
        new InstantCommand(
            () -> m_arm.setPistonRaised(true), m_arm));
    m_manipulator.povDown().onTrue(
        new InstantCommand(
            () -> m_arm.setPistonRaised(false), m_arm));

    
    m_manipulator.leftTrigger().onTrue(
        new InstantCommand(
            () -> m_intake.intakeIn(), m_intake));
    m_manipulator.leftTrigger().onFalse(
        new InstantCommand(
            m_intake::intakeStop, m_intake));

    
    m_manipulator.povLeft().onTrue(
        new InstantCommand(
            () -> m_arm.setElevatorExtended(false), m_arm));
    m_manipulator.povRight().onTrue(
        new InstantCommand(
            () -> m_arm.setElevatorExtended(true), m_arm));

    
    m_manipulator.start().onTrue(
        new InstantCommand(
            m_lights::setViolet, m_lights).andThen(
                new WaitCommand(OperatorConstants.kLightsTimeoutSeconds),
                new InstantCommand(() -> m_lights.teamChaser(m_safety_mode.getSelected()),m_lights)));
    
    
    m_manipulator.back().onTrue(
        new InstantCommand(
            m_lights::setYellow, m_lights).andThen(
                new WaitCommand(OperatorConstants.kLightsTimeoutSeconds),
                new InstantCommand(() -> m_lights.teamChaser(m_safety_mode.getSelected()),m_lights)));
    
    
    m_manipulator.leftBumper().onTrue(
        new InstantCommand(
        () -> m_arm.setElbowExtended(false), m_arm));

  }

    public void teleopInit() {
        m_arm.init();
        m_lights.init(m_safety_mode.getSelected());
    }
    public void autonomousInit() {
        m_arm.init();
        m_lights.init(m_safety_mode.getSelected());
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_autonomous_selecter.getSelected();
  }

  public Command getTankDriveCommand() {
    return new InstantCommand(
        () -> m_drivetrain.tankDrive(
            m_driverLeftJoystick.getY(),
            m_driverRightJoystick.getY(),
            m_driverRightJoystick.trigger().getAsBoolean(),
            m_driverLeftJoystick.trigger().getAsBoolean(),
            m_driverLeftJoystick.top().getAsBoolean(),
            m_driverRightJoystick.top().getAsBoolean(),
            m_safety_mode.getSelected()
        ),
        m_drivetrain
    );
  }

  public Command getIntakeCommand() {
    return new InstantCommand(
        () -> m_intake.checkPickedUp(m_objectInIntake), m_intake);
  }


  public Command getTestCommand() {
    return new AutonTest(m_drivetrain,m_arm,m_intake);
  }

  public void testInit() {
        m_arm.init();
        m_lights.init(m_safety_mode.getSelected());
    }
  
}