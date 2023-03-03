// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.button.Button;



public class Lights {
  PWMMotorController controller;

  public Lights() {
    controller = new Spark(9);

  }

  public void defaultColor() {
    controller.set(0.65);
  }
 
  public void rainbow() {
    controller.set(-0.99);
  }

  public void purple(){
    System.out.println("PURPLE!!!!!!!!!!!!");
    controller.set(0.91);
  }
  
  public void orangechase(){
    controller.set(0.01);
    //unfinished chaser code - matt
  }

  public void setLights(double in) {
    controller.set(in);
  }


}