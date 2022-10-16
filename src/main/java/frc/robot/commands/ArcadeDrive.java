/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.*;

public class ArcadeDrive extends Command {

  private ShuffleboardTab shuffTab = Shuffleboard.getTab("Robot7890");
  // private NetworkTableEntry shuffMessage = shuffTab.add("Drive Mode", "none").withSize(2, 1).withPosition(0, 1).getEntry();
  private NetworkTableEntry shuffSpeed = shuffTab.add("Speed", 0.0).withSize(1, 1).withPosition(0, 2).getEntry();
  private NetworkTableEntry shuffTurn = shuffTab.add("Turn", 0.0).withSize(1, 1).withPosition(1, 2).getEntry();
  // private NetworkTableEntry shuffVisionStatus = shuffTab.add("Vision Status", "inactive").withSize(2, 1).withPosition(0, 3).getEntry();
  private NetworkTableEntry shuffLeftMotors = shuffTab.add("Left Motors", 0.0).withSize(2, 1).withPosition(2, 2).getEntry();
  private NetworkTableEntry shuffRightMotors = shuffTab.add("Right Motors", 0.0).withSize(2, 1).withPosition(2, 3).getEntry();
  // private NetworkTableEntry shuffRobotInPosition = shuffTab.add("Vision In Position", false).withSize(2, 1).withPosition(6, 0).getEntry();
  // private NetworkTableEntry shuffTv = shuffTab.add("tv", 0.0).withSize(1, 1).withPosition(0, 4).getEntry();
  // private NetworkTableEntry shuffTx = shuffTab.add("tx", 0.0).withSize(1, 1).withPosition(1, 4).getEntry();
  // private NetworkTableEntry shuffTy = shuffTab.add("ty", 0.0).withSize(1, 1).withPosition(0, 5).getEntry();
  // private NetworkTableEntry shuffTa = shuffTab.add("ta", 0.0).withSize(1, 1).withPosition(1, 5).getEntry();
  // private NetworkTableEntry shuffTvFilter = shuffTab.add("tvFilter", 0.0).withSize(2, 1).withPosition(0, 6).getEntry();

  // boolean to indicate robot is in position
  boolean bRobotInPosition;
  double dThrottle;
  double dTurn;
  double dLeftMotors;
  double dRightMotors;
  boolean bVisionMode;
  double tv;
  double tx;
  double ty;
  double ta;
  double tvFilter;
  double dThrottleFilter;

  public ArcadeDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drivebase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);  // limelight in camera mode
    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);  // limelight leds off
    // shuffleboard message
   // shuffMessage.setString("Init");
    tvFilter = 0.0;
    dThrottleFilter = 0.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    //if (Robot.oi.getMainStart()) {
    //  bVisionMode = true;
    //}
    //if (Robot.oi.getMainBack()) {
    //  bVisionMode = false;
    //}
    // limelight variables from vision identification system
    //tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);  // target acquired yes =1 or no =0
    //tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);  // horizontal offset angle -27 to +27 deg
    //ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);  // vertical offset angle -20.5 to 20.5 deg
    //ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);  // target area 0 to 100%

    //tvFilter = 0.7 * tvFilter + 0.3 * tv;

    // if (bVisionMode) {
    //   NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);  // limelight in tracking mode
    //   NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);  // limelight leds on
    //   if (tvFilter > 0.5) {  // tv of 1 means target is acquired
    //     if (Math.abs(tx) < 0.5) {  // low tx means target is straight ahead
    //       if (ta < 3.0) {
    //         dThrottle = (3.0 - ta) * 0.1;
    //       }
    //       else {
    //         dThrottle = 0.07;
    //       }
    //       dTurn = 0.0;
    //       if (ta >= 2.7) {
    //         bRobotInPosition = true;
    //         bVisionMode = false;
    //       }
    //     }
    //     else {
    //       dThrottle = 0.15;
    //       dTurn = tx * 0.009;
    //     }
    //     dThrottleFilter = 0.7 * dThrottleFilter + 0.3 * dThrottle;
    //     dLeftMotors = dThrottleFilter + dTurn;
    //     dRightMotors = dThrottleFilter - dTurn;
    //     Robot.drivebase.setMotors(dLeftMotors, dRightMotors);   // runs robot in reverse (and turns in reverse correctly)
    //     //shuffVisionStatus.setString("ACTIVE");
    //   }
    //   else {  // target is not in view - go back to joystick control right away
    //     bVisionMode = false;
    //   }
    // }
    //else {
    //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(1);  // limelight in camera mode
    //  NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);  // limelight leds off
      dThrottle = 1.0 - (0.65 * Robot.oi.getMainRightTrigger());
      //dTurn = Robot.oi.getMainLeftJoyY() == 0.0 ? Robot.oi.getMainRightJoyX() * 0.35 : Robot.oi.getMainRightJoyX() * 0.22;
      if (Robot.oi.getMainLeftJoyY() < 0.05)
      {
        dTurn = Robot.oi.getMainRightJoyX() * 0.35;
      }
      else
      {
        dTurn = Robot.oi.getMainRightJoyX() * 0.15;
      }

      dLeftMotors = (Robot.oi.getMainLeftJoyY() + dTurn) * dThrottle;

      dRightMotors = (Robot.oi.getMainLeftJoyY() - dTurn) * dThrottle;
      Robot.drivebase.setMotors(dLeftMotors, dRightMotors);
      //shuffVisionStatus.setString("inactive");
      //if (Robot.oi.getMainBack()) {
      //  bRobotInPosition = false;
      //}
    //}

    // shuffleboard message
    //shuffMessage.setString("Execute");
    shuffSpeed.setDouble(dThrottle);
    shuffTurn.setDouble(dTurn);
    shuffLeftMotors.setDouble(dLeftMotors);
    shuffRightMotors.setDouble(dRightMotors);
    //shuffTv.setDouble(tv);
    //shuffTvFilter.setDouble(tvFilter);
    //shuffTx.setDouble(tx);
    //shuffTy.setDouble(ty);
    //shuffTa.setDouble(ta);
    //shuffRobotInPosition.setBoolean(bRobotInPosition);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //shuffMessage.setString("End");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
