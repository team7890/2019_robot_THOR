/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

//import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.command.Subsystem;
//import com.ctre.phoenix.*;
import frc.robot.commands.ArcadeDrive;

import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Drivebase extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private final VictorSPX leftMotorOne;
  private final VictorSPX leftMotorTwo;
  private final VictorSPX rightMotorOne;
  private final VictorSPX rightMotorTwo;

  private final boolean bBRAKE_MODE = true;
  private final double dRAMP = 0.30;

  // private final SpeedControllerGroup leftGroup;
  // private final SpeedControllerGroup rightGroup;

  //private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
  //private NetworkTableEntry bDriveMode = tab.add("Drive Mode", 1).getEntry();


  public Drivebase() {

    leftMotorOne = new VictorSPX(RobotMap.LEFT_MOTOR_ONE.value);
    leftMotorTwo = new VictorSPX(RobotMap.LEFT_MOTOR_TWO.value);
    rightMotorOne = new VictorSPX(RobotMap.RIGHT_MOTOR_ONE.value);
    rightMotorTwo = new VictorSPX(RobotMap.RIGHT_MOTOR_TWO.value);

    // init all drive motors; brake mode is false (i.e. coast) and ramp is 1.0 sec from neutral to full speed
    Robot.initVictor(leftMotorOne, bBRAKE_MODE, dRAMP);
    Robot.initVictor(leftMotorTwo, bBRAKE_MODE, dRAMP);
    Robot.initVictor(rightMotorOne, bBRAKE_MODE, dRAMP);
    Robot.initVictor(rightMotorTwo, bBRAKE_MODE, dRAMP);

    leftMotorTwo.follow(leftMotorOne);
    rightMotorTwo.follow(rightMotorOne);
    // leftGroup = new SpeedControllerGroup(leftMotorOne, leftMotorTwo);
    // rightGroup = new SpeedControllerGroup(rightMotorOne, rightMotorTwo);
  }

  // public void setBrakeMode() {
  //   leftMotorOne.setNeutralMode(NeutralMode.Brake);
  //   rightMotorOne.setNeutralMode(NeutralMode.Brake);
  //   leftMotorTwo.setNeutralMode(NeutralMode.Brake);
  //   rightMotorTwo.setNeutralMode(NeutralMode.Brake);
  // }

  // public void setCoastMode() {
  //   leftMotorOne.setNeutralMode(NeutralMode.Coast);
  //   rightMotorOne.setNeutralMode(NeutralMode.Coast);
  //   leftMotorTwo.setNeutralMode(NeutralMode.Coast);
  //   rightMotorTwo.setNeutralMode(NeutralMode.Coast);
  // }

  public void setMotors(double left, double right) {
   leftMotorOne.set(ControlMode.PercentOutput, left);
   rightMotorOne.set(ControlMode.PercentOutput, -right);
  }

  // public void initDriveMotor(VictorSPX motor) {
  //   motor.setNeutralMode(NeutralMode.Coast);
	// 	motor.neutralOutput();
	// 	motor.setSensorPhase(false);
	// 	motor.configNominalOutputForward(0.0, 0);
  //   motor.configNominalOutputReverse(0.0, 0);
  // }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new ArcadeDrive());
  }
}
