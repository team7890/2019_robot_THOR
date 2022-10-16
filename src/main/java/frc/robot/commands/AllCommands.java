/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.*;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class AllCommands extends Command {

  // constants
  private final int iHATCH_EXTEND_SCANS = 10;          // how many 20 ms scans to wait after extending pneumatics to move elevator
  private final double dHATCH_ELEV_MOVE = 3000.0;     // how far to move elevator (encoder counts)

  private final double dELEVATOR_TOLERANCE = 400.0;   // how close is good enough (encoder counts) to consider elevator at the right height
  // private final double dTILT_TOLERANCE = 200.0;       // how close is good enough (encoder counts) for cargo tilt

  private final double dCARGO_HIGH_TARGET = -41000.0;      // cargo high elevator height
  private final double dCARGO_MIDDLE_TARGET = -45000.0;    // cargo middle elevator height
  private final double dCARGO_LOW_TARGET = -12500.0;       // cargo low elevator height
  private final double dHATCH_HIGH_TARGET = -73500.0;      // hatch high elevator height
  private final double dHATCH_MIDDLE_TARGET = -50400.0;    // hatch middle elevator height
  private final double dHATCH_LOW_TARGET = -12500.0;        // hatch low elevator height
  private final double dELEVATOR_HOME = 0.0;              // elevator home height

  // private final double dTILT_HIGH = 8000.0;         // tilt highest position
  // private final double dTILT_HORIZ = 0.0;           // tilt horizontal position
  // private final double dTILT_LOW = -8000.0;         // tilt lowest position

  private final double dELEV_MAN_SPEED_UP = 0.8;
  private final double dELEV_MAN_SPEED_DOWN = 0.5;
  private final double dTILT_MAN_SPEED = 0.3;
  private final double dCARGO_LOAD_SPEED = 0.7;
  private final double dCARGO_SHOOT_FAST_SPEED = 0.9;
  private final double dCARGO_SHOOT_SLOWLY_SPEED = 0.4;
  private final double dCARGO_HELP_SPEED = 0.5;

  // pneumatics variables
  private boolean bHatchExtend;
  private boolean bLiftFront;
  private boolean bLiftBack;
  private boolean bLiftHelper;
  private boolean bLiftHelperOld;
  private boolean bHelperToggle;
  private boolean bAutoHatchLoad;
  private boolean bAutoHatchPlace;
  private boolean bToggleHatch;
  private boolean bToggleHatchOld;
  private int iScanCount;

  // elevator variables
  private double dElevatorLiftCommand;
  private double dElevatorActualPosition;
  private double dElevatorDesiredPosition;
  private double dElevatorControl;
  private boolean bElevatorInPosition;
  private boolean bHatchInAuto;
  // elevator bottom switch
  DigitalInput limitSwitch = new DigitalInput(1);

  // cargo variables
  private boolean bCargoLastActionLoad;
  // private boolean bCargoTiltHitTop;
  private boolean bCargoLoad;
  private boolean bCargoShootFast;
  private boolean bCargoShootSlowly;
  // private double dCargoTiltCurrent;
  private double dCargoTiltSpeed;
  //public static PowerDistributionPanel pdp;
  private int iHelperCount;

  // cargo tilt variables
  // private double dTiltLiftCommand;
  // private double dTiltActualPosition;
  // private double dTiltDesiredPosition;
  private double dTiltControl;          // shows as not being used if not in shuffleboard - only a debug variable but keep it for now
  // private boolean bTiltInPosition;

  // shuffleboard setup tab
  private ShuffleboardTab shuffTab = Shuffleboard.getTab("Robot7890");

  // hatch shuffleboard
  private NetworkTableEntry shuffHatchLoad = shuffTab.add("Hatch Load", false).withSize(2, 1).withPosition(6, 1).getEntry();
  private NetworkTableEntry shuffHatchPlace = shuffTab.add("Hatch Place", false).withSize(2, 1).withPosition(6, 2).getEntry();

  // elevator shuffleboard
  //  private NetworkTableEntry shuffElevatorMode = shuffTab.add("Elevator Mode", "none").withSize(2, 1).withPosition(4, 0).getEntry();
  private NetworkTableEntry shuffElevatorActual = shuffTab.add("Elevator Actual", 0.0).withSize(2, 1).withPosition(4, 1).getEntry();
  private NetworkTableEntry shuffElevatorDesired = shuffTab.add("Elevator Desired", 0.0).withSize(2, 1).withPosition(4, 2).getEntry();
  //private NetworkTableEntry shuffElevatorControl = shuffTab.add("Elevator Control", 0.0).withSize(2, 1).withPosition(4, 3).getEntry();
  //private NetworkTableEntry shuffElevatorOutput = shuffTab.add("Elev Talon Output", 0.0).withSize(2, 1).withPosition(4, 4).getEntry();
  private NetworkTableEntry shuffElevInPos = shuffTab.add("Elev In Pos", false).withSize(2, 1).withPosition(6, 3).getEntry();
  private NetworkTableEntry shuffLimitSwitch = shuffTab.add("Limit Switch", false).withSize(1, 1).withPosition(6, 5).getEntry();

  // cargo shuffleboard
  // private NetworkTableEntry shuffCargoMode = shuffTab.add("Cargo Mode", "none").withSize(2, 1).withPosition(2, 0).getEntry();
  // private NetworkTableEntry shuffCargoLoad = shuffTab.add("Load", false).withSize(2, 1).withPosition(2, 1).getEntry();
  // private NetworkTableEntry shuffCargoShootFast = shuffTab.add("Shoot Fast", false).withSize(2, 1).withPosition(2, 2).getEntry();
  // private NetworkTableEntry shuffCargoShootSlowly = shuffTab.add("Shoot Slowly", false).withSize(2, 1).withPosition(2, 3).getEntry();
  // private NetworkTableEntry shuffCargoTiltSpeed = shuffTab.add("Tilt Speed", 0.0).withSize(2, 1).withPosition(4, 3).getEntry();
  // private NetworkTableEntry shuffCargoTiltHitTop = shuffTab.add("Tilt Hit Top", false).withSize(2, 1).withPosition(4, 4).getEntry();
  // private NetworkTableEntry shuffCargoTiltCurrent = shuffTab.add("Tilt Current", 0.0).withSize(2, 1).withPosition(4, 5).getEntry();
  // private NetworkTableEntry shuffCargoTiltActual = shuffTab.add("Tilt Actual", 0.0).withSize(2, 1).withPosition(2, 4).getEntry();
  // private NetworkTableEntry shuffCargoTiltDesired = shuffTab.add("Tilt Desired", 0.0).withSize(2, 1).withPosition(2, 5).getEntry();
  private NetworkTableEntry shuffCargoTiltControl = shuffTab.add("Tilt Control", 0.0).withSize(2, 1).withPosition(2, 6).getEntry();
  // private NetworkTableEntry shuffTiltInPos = shuffTab.add("Tilt In Pos", false).withSize(2, 1).withPosition(6, 4).getEntry();

  public AllCommands() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.allsystems);
    //pdp = new PowerDistributionPanel();
    resetEncoders();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
    dElevatorDesiredPosition = dElevatorActualPosition;
  }
  
  public void resetEncoders() {
    Robot.allsystems.setElevatorPosition(0);
    dElevatorDesiredPosition = 0.0;
    Robot.allsystems.setTiltPosition(0);
    // dTiltDesiredPosition = 0.0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    bHatchInAuto = Robot.oi.getMainLBumper() || bAutoHatchLoad || Robot.oi.getMainRBumper() || bAutoHatchPlace;

    // ==== pneumatics actions ====
    //    hatch manual extend
    if (! bHatchInAuto) {
      bToggleHatch = Robot.oi.getMainXButton();
      if (bToggleHatch && !bToggleHatchOld) {
        bHatchExtend = !bHatchExtend;
      }

      
      Robot.allsystems.extendHatch(bHatchExtend);  
      bToggleHatchOld = bToggleHatch;
    }

    //    endgame lift front of robot
    bLiftFront = Robot.oi.getMechStart(); 
    Robot.allsystems.liftFront(bLiftFront);
 
    //    endgame lift back of robot
    bLiftBack = Robot.oi.getMechBack();
    Robot.allsystems.liftBack(bLiftBack);

    //    cargo helper lift
    iHelperCount = iHelperCount + 1;
    bLiftHelper = Robot.oi.getMechYButton();
    if (bLiftHelper != bLiftHelperOld && bLiftHelper && iHelperCount > 20) {
      bHelperToggle = !bHelperToggle;
      iHelperCount = 0;
    }
    Robot.allsystems.grabHelper(bHelperToggle);

    //    auto mode -- Load Hatch
    if (Robot.oi.getMainLBumper() || bAutoHatchLoad) {
      if (! bAutoHatchLoad) {
        Robot.allsystems.extendHatch(true);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        dElevatorDesiredPosition = dElevatorActualPosition - 2.5 * dHATCH_ELEV_MOVE;
        iScanCount = 0;
        bAutoHatchLoad = true;
      }
      else {
        iScanCount = iScanCount + 1;
        Robot.allsystems.extendHatch(true);
        if (iScanCount > iHATCH_EXTEND_SCANS) {
          dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
          dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
          if (Math.abs(dElevatorActualPosition - dElevatorDesiredPosition) < dELEVATOR_TOLERANCE) {
            Robot.allsystems.extendHatch(false);
            bAutoHatchLoad = false;
          }
        }
      }
    }
    
    //    auto mode -- Place Hatch
    if (Robot.oi.getMainRBumper() || bAutoHatchPlace) {
      if (! bAutoHatchPlace) {
        Robot.allsystems.extendHatch(true);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        dElevatorDesiredPosition = Math.min(dElevatorActualPosition + dHATCH_ELEV_MOVE, 0.0);
        iScanCount = 0;
        bAutoHatchPlace = true;
      }
      else {
        iScanCount = iScanCount + 1;
        Robot.allsystems.extendHatch(true);
        if (iScanCount > iHATCH_EXTEND_SCANS) {
          dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
          dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
          if (Math.abs(dElevatorActualPosition - dElevatorDesiredPosition) < dELEVATOR_TOLERANCE) {
            Robot.allsystems.extendHatch(false);
            bAutoHatchPlace = false;
          }
        }
      }
    }

    //cancel auto hatch
    if (Robot.oi.getMainYButton()) {
      bAutoHatchPlace = false;
      bAutoHatchLoad = false;
    }

    // ==== END of pneumatics actions ====

    // ==== elevator actions ====
    //    only do elevator actions if not loading or placing a hatch (driver bumpers)
    if (! bHatchInAuto) {
      //    manual mode -- Mech Start + Mech Left Joy Y
      if (Robot.oi.getMechRBumper()) {
        if (Robot.oi.getMechLeftJoyY() > 0.0) {
          dElevatorLiftCommand = - Robot.oi.getMechLeftJoyY() * dELEV_MAN_SPEED_UP;
        }
        else {
          dElevatorLiftCommand = - Robot.oi.getMechLeftJoyY() * dELEV_MAN_SPEED_DOWN;
        }
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(0, false, dElevatorLiftCommand);
        dElevatorDesiredPosition = Robot.allsystems.getElevatorPosition();
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Manual");
      }
      //    auto mode -- Cargo High
      else if (Robot.oi.getCargoH()) {
        dElevatorDesiredPosition = dCARGO_HIGH_TARGET;
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Auto Cargo H");
        // dTiltDesiredPosition = dTILT_HIGH;
        // dTiltControl = Robot.allsystems.gotoTiltPosition(dTiltDesiredPosition, true, dTiltLiftCommand);
        // dTiltActualPosition = Robot.allsystems.getTiltPosition();
      }
      //    auto mode -- Cargo Middle
      else if (Robot.oi.getCargoM()) {
        dElevatorDesiredPosition = dCARGO_MIDDLE_TARGET;
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Auto Cargo M");
        // dTiltDesiredPosition = dTILT_HORIZ;
        // dTiltControl = Robot.allsystems.gotoTiltPosition(dTiltDesiredPosition, true, dTiltLiftCommand);
        // dTiltActualPosition = Robot.allsystems.getTiltPosition();
      }
      //    auto mode -- Cargo Low
      else if (Robot.oi.getCargoL()) {
        dElevatorDesiredPosition = dCARGO_LOW_TARGET;
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Auto Cargo L");
        // dTiltDesiredPosition = dTILT_HORIZ;
        // dTiltControl = Robot.allsystems.gotoTiltPosition(dTiltDesiredPosition, true, dTiltLiftCommand);
        // dTiltActualPosition = Robot.allsystems.getTiltPosition();
      }
      //    auto mode -- Hatch High
      else if (Robot.oi.getHatchH()) {
        dElevatorDesiredPosition = dHATCH_HIGH_TARGET;
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Auto Hatch H");
      }
      //    auto mode -- Hatch Middle
      else if (Robot.oi.getHatchM()) {
        dElevatorDesiredPosition = dHATCH_MIDDLE_TARGET;
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Auto Hatch M");
      }
      //    auto mode -- Hatch Low
      else if (Robot.oi.getHatchL()) {
        dElevatorDesiredPosition = dHATCH_LOW_TARGET;
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Auto Hatch L");
      }
      //    auto mode -- send elevator home
      else if (Robot.oi.getElevatorHome()) {
        dElevatorDesiredPosition = dELEVATOR_HOME;
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Auto Home");
      }

      //    auto mode -- hold last position
      else
      {
        dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
        dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
        //shuffElevatorMode.setString("Hold in Place");
      }
    }
    //if hatch auto then need to drive elevator to desired position
    else {
      dElevatorControl = Robot.allsystems.gotoElevatorPosition(dElevatorDesiredPosition, true, dElevatorLiftCommand);
      dElevatorActualPosition = Robot.allsystems.getElevatorPosition();
      //shuffElevatorMode.setString("Hold in Place");
    }

    //    reset encoder position to zero (if necessary in middle of match)
    if (Robot.oi.getElevatorReset()) {
      Robot.allsystems.setElevatorPosition(0);
      dElevatorDesiredPosition = 0.0;
    }

    //    set in position light for shuffleboard display on whether elevator is in position or not
    if (Math.abs(dElevatorActualPosition - dElevatorDesiredPosition) < dELEVATOR_TOLERANCE) {
      bElevatorInPosition = true;
    }
    else {
      bElevatorInPosition = false;
    }
    // ==== END of elevator actions ====

    // ==== cargo actions ====
    //    intake or outtake cargo
    bCargoLoad = Robot.oi.getMechLBumper();
    bCargoShootFast = Robot.oi.getMainAButton();
    bCargoShootSlowly = Robot.oi.getMainBButton();
    if (bCargoLoad) {
      Robot.allsystems.setCargoGrabMotors(-dCARGO_LOAD_SPEED);  // at this speed, ball intakes into grabber
      Robot.allsystems.setCargoHelpMotor(dCARGO_HELP_SPEED);
      bCargoLastActionLoad = true;
      // dTiltDesiredPosition = dTILT_LOW;
      // dTiltControl = Robot.allsystems.gotoTiltPosition(dTiltDesiredPosition, true, dTiltLiftCommand);
      // dTiltActualPosition = Robot.allsystems.getTiltPosition();
    }
    else {
      if (bCargoShootFast || bCargoShootSlowly) {
        if (bCargoShootFast) {
          Robot.allsystems.setCargoGrabMotors(dCARGO_SHOOT_FAST_SPEED);  // at this speed, ball shoots 12" or more out of grabber
        }
        else {
          Robot.allsystems.setCargoGrabMotors(dCARGO_SHOOT_SLOWLY_SPEED);  // at this speed, the ball will dribble out of the grabber instead of shooting
        }
        bCargoLastActionLoad = false;
      }
      else {
        if (bCargoLastActionLoad) {
          Robot.allsystems.setCargoGrabMotors(-0.08);  // at this speed, ball stays in grabber without falling out
        }
        else {
          Robot.allsystems.setCargoGrabMotors(0.0);
        }    
      }
      Robot.allsystems.setCargoHelpMotor(0.0);
    }

    //    set tilt of cargo grabber manually
    // dCargoTiltCurrent = 0;
    dCargoTiltSpeed = Robot.oi.getMechRightJoyY();
    Robot.allsystems.setCargoTiltMotor(dCargoTiltSpeed * dTILT_MAN_SPEED);
    dTiltControl = dCargoTiltSpeed * dTILT_MAN_SPEED;
    
    if (Math.abs(dCargoTiltSpeed) < 0.05) {
      Robot.allsystems.setCargoTiltMotor(-0.05);
      dTiltControl = -0.05;
    }
    else {
      if (dCargoTiltSpeed < -0.05) {
      Robot.allsystems.setCargoTiltMotor(dCargoTiltSpeed * dTILT_MAN_SPEED * 2.0);
      dTiltControl = dCargoTiltSpeed * dTILT_MAN_SPEED;
      }
      else {
      Robot.allsystems.setCargoTiltMotor(dCargoTiltSpeed * dTILT_MAN_SPEED);
      dTiltControl = dCargoTiltSpeed * dTILT_MAN_SPEED;
      }
    }
    // if (dCargoTiltSpeed > 0.05) {
    //   Robot.allsystems.setCargoTiltMotor(dCargoTiltSpeed * dTILT_MAN_SPEED);
    //   bCargoTiltHitTop = false;
    // }
    // else {
    //   if (dCargoTiltSpeed < -0.05) {
    //     if (dCargoTiltCurrent > 10.5) { 
    //       bCargoTiltHitTop = true;
    //     }
    //     if (bCargoTiltHitTop == false) {
    //       Robot.allsystems.setCargoTiltMotor(dCargoTiltSpeed * dTILT_MAN_SPEED);
    //     }
    //     else {
    //       Robot.allsystems.setCargoTiltMotor(0.0);
    //     }
    //   }
    //   else {
    //     Robot.allsystems.setCargoTiltMotor(0.0);
    //   }
    // }

    //    set in position light for shuffleboard display on whether tilt is in position or not
    // if (Math.abs(dTiltActualPosition - dTiltDesiredPosition) < dTILT_TOLERANCE) {
    //   bTiltInPosition = true;
    // }
    // else {
    //   bTiltInPosition = false;
    // }
    // END of ==== cargo actions ====

    // ==== shuffleboard messages ====
    // hatch
    shuffHatchLoad.setBoolean(bAutoHatchLoad);
    shuffHatchPlace.setBoolean(bAutoHatchPlace);
    // elevator
    shuffElevatorActual.setDouble(dElevatorActualPosition);
    shuffElevatorDesired.setDouble(dElevatorDesiredPosition);
    //  shuffElevatorControl.setDouble(dElevatorControl);
    //  shuffElevatorOutput.setDouble(Robot.allsystems.getElevatorOutput());
    shuffElevInPos.setBoolean(bElevatorInPosition);
    shuffLimitSwitch.setBoolean(limitSwitch.get());
    // cargo
    //  shuffCargoMode.setString("Execute");
    //  shuffCargoLoad.setBoolean(bCargoLoad);
    //  shuffCargoShootFast.setBoolean(bCargoShootFast);
    //  shuffCargoShootSlowly.setBoolean(bCargoShootSlowly);
    //  shuffCargoTiltSpeed.setDouble(dCargoTiltSpeed);
    //  shuffCargoTiltHitTop.setBoolean(bCargoTiltHitTop);
    //  shuffCargoTiltCurrent.setDouble(dCargoTiltCurrent);
    //  shuffCargoTiltActual.setDouble(dTiltActualPosition);  
    //  shuffCargoTiltDesired.setDouble(dTiltDesiredPosition);  
    shuffCargoTiltControl.setDouble(dTiltControl);  
    //  shuffTiltInPos.setBoolean(bTiltInPosition);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
