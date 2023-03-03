// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * A {@link Button} that gets its state from a POV on a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class ButtonAndDpad extends Button {
  private final POVButton m_pov;
  private final Joystick m_joystick;
  private final int m_joystickButton;
  
  

  /**
   * Creates a POV button for triggering commands.
   *
   * @param joystick The GenericHID object that has the POV
   * @param angle The desired angle in degrees (e.g. 90, 270)
   * @param povNumber The POV number (see {@link GenericHID#getPOV(int)})
   */
  public ButtonAndDpad(Joystick joystick, int joystickButton, POVButton pov) {
    requireNonNullParam(joystick, "joystick", "joystick");
    requireNonNullParam(pov, "joystick", "POVButton");


    m_pov = pov;
    m_joystick = joystick;
    m_joystickButton = joystickButton;
    
    
  }

  
  /**
   * Checks whether the current value of the POV is the target angle.
   *
   * @return Whether the value of the POV matches the target angle
   */
  @Override
  public boolean get() {
    return (m_joystick.getRawButton(m_joystickButton) && m_pov.get() );
  }
}