/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

public enum RobotMap {

  //CAN motor controller mappings
  LEFT_MOTOR_ONE(1),
  LEFT_MOTOR_TWO(2),
  RIGHT_MOTOR_ONE(3),
  RIGHT_MOTOR_TWO(4),
  GRABBER_MOTOR_ONE(5),
  GRABBER_MOTOR_TWO(6),
  CARGOTILT_MOTOR(7),
  ELEVATOR_MOTOR(8),
  CARGOHELP_MOTOR(9),

  //Controller Mapping
  MAIN_CONTROLLER(0),
  MECH_CONTROLLER(1),
  USB_BUTTON_BOX(2);

  // PCM Mapping
  //PCM_ONE(9);

  public final int value;

	RobotMap(int value) {
  	this.value = value;
	} 

}
