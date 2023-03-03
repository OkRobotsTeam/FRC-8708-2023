// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A {@link Button} that gets its state from a POV on a {@link GenericHID}.
 *
 * <p>This class is provided by the NewCommands VendorDep
 */
public class TriggerAsButton extends Button {
  private final XboxController m_controller;
  private final int m_triggerNumber;
  private final double m_minValue;

  /**
   * Creates a POV button for triggering commands.
   *
   * @param joystick The GenericHID object that has the POV
   * @param angle The desired angle in degrees (e.g. 90, 270)
   * @param povNumber The POV number (see {@link GenericHID#getPOV(int)})
   */
  public TriggerAsButton(XboxController controller, int triggerNumber, double minValue) {
    requireNonNullParam(controller, "joystick", "POVButton");

    m_controller = controller;
    m_triggerNumber = triggerNumber;
    m_minValue=minValue;
  }

  
  /**
   * Checks whether the current value of the POV is the target angle.
   *
   * @return Whether the value of the POV matches the target angle
   */
  @Override
  public boolean get() {
    if (m_triggerNumber == 0 ) {
      return m_controller.getLeftTriggerAxis() > m_minValue;
    } else {
      return m_controller.getRightTriggerAxis() > m_minValue;
    }
  }
}