// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} that gets its state from a POV on a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class DoubleButtonPress extends Button {
  private final XboxController m_controller;
  private final int m_controllerButton;
  private final Joystick m_joystick;
  private final int m_joystickButton;
  
  

  /**
   * Creates a POV button for triggering commands.
   *
   * @param joystick The GenericHID object that has the POV
   * @param angle The desired angle in degrees (e.g. 90, 270)
   * @param povNumber The POV number (see {@link GenericHID#getPOV(int)})
   */
  public DoubleButtonPress(XboxController controller, int controllerButton, Joystick joystick, int joystickButton) {
    requireNonNullParam(controller, "joystick", "POVButton");

    m_controller = controller;
    m_joystick = joystick;
    m_joystickButton = joystickButton;
    m_controllerButton = controllerButton;
    
    
  }

  
  /**
   * Checks whether the current value of the POV is the target angle.
   *
   * @return Whether the value of the POV matches the target angle
   */
  @Override
  public boolean get() {
    return (m_joystick.getRawButton(m_joystickButton) && m_controller.getRawButton(m_controllerButton) );
  }
}