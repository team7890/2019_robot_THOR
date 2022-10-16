/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.buttons.JoystickButton;
// import frc.robot.commands.LiftElevator;
// import edu.wpi.first.wpilibj.buttons.Button;

public class OI {

  private static final double DEADZONE = 0.15 ;

  private Joystick MAIN_CONTROLLER;
  private Joystick MECH_CONTROLLER;
  private Joystick USB_BUTTON_BOX;

  //private final Button elevatorUp;
  //private final Button elevatorDown;
  //private final Button elevatorActuate;
  //private Joystick ARM_CONTROLLER;
  //private double value;

  public OI() { 
    MAIN_CONTROLLER = new Joystick(RobotMap.MAIN_CONTROLLER.value);
    MECH_CONTROLLER = new Joystick(RobotMap.MECH_CONTROLLER.value);
    USB_BUTTON_BOX = new Joystick(RobotMap.USB_BUTTON_BOX.value);
    //elevatorActuate = new JoystickButton(MECH_CONTROLLER, 6);
    //elevatorDown = new JoystickButton(MAIN_CONTROLLER, 5);
    //elevatorDown.whileHeld(new LiftElevator(0.25));
    //elevatorActuate.whileHeld(new LiftElevator(0.45));
  }


  // ========== MAIN CONTROLLER FOR DRIVER ========== //


  // no robot function
  public double getMainLeftJoyX() {
    double value = MAIN_CONTROLLER.getRawAxis(0);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  // speed forward and backward
  public double getMainLeftJoyY() {
    double value = MAIN_CONTROLLER.getRawAxis(1);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }
  
  // no robot function
  public double getMainLeftTrigger() {
    double value = MAIN_CONTROLLER.getRawAxis(2);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    //return Math.abs(value) > DEADZONE ? value : 0.0;
  }

  // no robot function
  public double getMainRightTrigger() {
    double value = MAIN_CONTROLLER.getRawAxis(3);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
    //return Math.abs(value) > DEADZONE ? value : 0.0;
  }

  // turn robot right left
  public double getMainRightJoyX() {
    double value = MAIN_CONTROLLER.getRawAxis(4);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  // no robot function
  public double getMainRightJoyY() {
    double value = MAIN_CONTROLLER.getRawAxis(5);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  // cargo shoot fast
  public boolean getMainAButton() {
    return MAIN_CONTROLLER.getRawButton(1);
  }

  // cargo shoot slowly
  public boolean getMainBButton() {
    return MAIN_CONTROLLER.getRawButton(2);
  }

  // hatch extend
  public boolean getMainXButton() {
    return MAIN_CONTROLLER.getRawButton(3);
  }

  // cancel auto hatch
  public boolean getMainYButton() {
    return MAIN_CONTROLLER.getRawButton(4);
  }

  // hatch load SeQuEnCe - extend hatch, raise elevator, retract hatch.
  public boolean getMainLBumper() {
    return MAIN_CONTROLLER.getRawButton(5);
  }

  // hatch place SeQuEnCe - raise elevator, extend hatch, lower elevator, retract hatch.
  public boolean getMainRBumper() {
    return MAIN_CONTROLLER.getRawButton(6);
  }

  // no robot function
  public boolean getMainBack() {
    return MAIN_CONTROLLER.getRawButton(7);
  }

  // start vision mode driving
  public boolean getMainStart() {
    return MAIN_CONTROLLER.getRawButton(8);
  }
  


  // ========== MECH CONTROLLER FOR COPILOT ========== //



  // elevator up and down
  public double getMechLeftJoyY() {
    double value = - MECH_CONTROLLER.getRawAxis(1);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  // grabber tilt up and down
  public double getMechRightJoyY() {
    double value = MECH_CONTROLLER.getRawAxis(5);
    return Math.abs(value) > DEADZONE ? ((Math.abs(value) - DEADZONE) * Math.abs(value) / (0.85 * value)) : 0.0;
  }

  // no robot function
  public boolean getMechAButton() {
    return MECH_CONTROLLER.getRawButton(1);
  }

  // no robot function
  public boolean getMechBButton() {
    return MECH_CONTROLLER.getRawButton(2);
  }
  
  // no robot function
  public boolean getMechXButton() {
    return MECH_CONTROLLER.getRawButton(3);
  }

  // no robot function
  public boolean getMechYButton() {
    return MECH_CONTROLLER.getRawButton(4);
  }

  // cargo intake
  public boolean getMechLBumper() {
    return MECH_CONTROLLER.getRawButton(5);
  }

  // no robot function
  public boolean getMechRBumper() {
    return MECH_CONTROLLER.getRawButton(6);
  }

  // back of robot climb pneumatics
  public boolean getMechBack() {
    return MECH_CONTROLLER.getRawButton(7);
  }

  // front of robot climb pneumatics
  public boolean getMechStart() {
    return MECH_CONTROLLER.getRawButton(8);
  }


  // ========== USB_BUTTON_BOX ========== //


  public boolean getCargoH() {
    return USB_BUTTON_BOX.getRawButton(1);
  }

  public boolean getCargoM() {
    return USB_BUTTON_BOX.getRawButton(2);
  }

  public boolean getCargoL() {
    return USB_BUTTON_BOX.getRawButton(3);

  }
  public boolean getHatchH() {
    return USB_BUTTON_BOX.getRawButton(4);
  }

  public boolean getHatchM() {
    return USB_BUTTON_BOX.getRawButton(5);
  }

  public boolean getHatchL() {
    return USB_BUTTON_BOX.getRawButton(6);
  }

  public boolean getElevatorReset() {
    return USB_BUTTON_BOX.getRawButton(7);
  }

  public boolean getElevatorHome() {
    return USB_BUTTON_BOX.getRawButton(8);
  }
}