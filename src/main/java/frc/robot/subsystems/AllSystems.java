/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

// elevator importsvs
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
// pneumatics imports
import edu.wpi.first.wpilibj.command.Subsystem;
// frc robot imports
import frc.robot.Robot;
import frc.robot.RobotMap;// pneumatics imports
import frc.robot.commands.AllCommands;

/**
 * Add your docs here.
 */
public class AllSystems extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Constants
  private final double dELEVATOR_TOP_POSITION = -79000.0;
  private final double dELEVATOR_BOTTOM_POSITION = 0.0;
  private final double dTILT_TOP_POSITION = 16000.0;
  private final double dTILT_BOTTOM_POSITION = -16000.0;

  private final double d_kP = 1.0 / 40000.0;
  private final double d_kD = 1.0 / 2500.0;
  private final double d_kI = 0.0;
  private final double d_PEAK_OUTPUT = 0.5;
  private final double d_CLOSED_LOOP_RAMP = 1.0;
  private final double d_OPEN_LOOP_RAMP = 0.2;
  private final boolean b_ELEV_SENSOR_PHASE = false;     // must be set so that + motor direction equals + elevator direction

  // Pneumatics stuff
  private final Compressor CompressorOne;
  private final Solenoid hatchCylinder;
  private final Solenoid frontLift;
  private final Solenoid backLift;
  private final Solenoid helperLift;

  // Elevator stuff
  private final TalonSRX ElevatorOne;

  private double dControl;
  private double dControlOld;
  private double dControlLimited;
  private double dControlDelta;
  private double dActualPosition;
  private double dActualPositionOld;
  private double dError;
  private double dDeriv;

  private final static boolean bElevMode = true;  // false for Talon elevator PID, true for VS Code PID control


  // Cargo stuff
  private final VictorSPX CargoGrabOne;
  private final VictorSPX CargoGrabTwo;
  private final TalonSRX CargoTiltOne;
  private final VictorSPX CargoHelpOne;
  
  private double dTiltControl;
  private double dTiltControlOld;
  private double dTiltControlLimited;
  private double dTiltControlDelta;
  private double dTiltActualPosition;
  private double dTiltActualPositionOld;
  private double dTiltError;
  private double dTiltDeriv;

  public AllSystems() {
    
    // pneumatics initializations
    CompressorOne = new Compressor();
    hatchCylinder = new Solenoid(0);
    frontLift = new Solenoid(1);
    backLift = new Solenoid(2);
    helperLift = new Solenoid(3);
    
    // elevator initializations
    ElevatorOne = new TalonSRX(RobotMap.ELEVATOR_MOTOR.value);
    // init elevator motor; brake is true (so it doesn't come crashing down) and ramp is 0.3 sec from neutral to full speed
    Robot.initTalon(ElevatorOne, true, 0.3);
    // init some new values for elevator motor and overwrite some existing in initTalon
    ElevatorOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    ElevatorOne.setSelectedSensorPosition(0);
    ElevatorOne.setSensorPhase(b_ELEV_SENSOR_PHASE);
    ElevatorOne.configNominalOutputForward(0, 10);
    ElevatorOne.configNominalOutputReverse(0, 10);
    // ElevatorOne.setNeutralMode(NeutralMode.Brake);  // already done in initTalon
    ElevatorOne.configClosedLoopPeakOutput(0, d_PEAK_OUTPUT);
    ElevatorOne.configOpenloopRamp(d_OPEN_LOOP_RAMP);
    ElevatorOne.configClosedloopRamp(d_CLOSED_LOOP_RAMP);
    ElevatorOne.config_kP(0, d_kP, 100);
    ElevatorOne.config_kD(0, d_kD, 100);
    ElevatorOne.config_kI(0, d_kI, 100);
    //ElevatorOne.configForwardSoftLimitThreshold(forwardSensorLimit)
    dControl = 0.0;

    // cargo initializations
    CargoGrabOne = new VictorSPX(RobotMap.GRABBER_MOTOR_ONE.value);
    CargoGrabTwo = new VictorSPX(RobotMap.GRABBER_MOTOR_TWO.value);
    CargoTiltOne = new TalonSRX(RobotMap.CARGOTILT_MOTOR.value);
    CargoHelpOne = new VictorSPX(RobotMap.CARGOHELP_MOTOR.value);
    
    // init all cargo motors; brake mode is true and ramp is limited to a fast 0.1 sec from neutral to full speed
    Robot.initVictor(CargoGrabOne, true, 0.1);
    Robot.initVictor(CargoGrabTwo, true, 0.1);
    Robot.initTalon(CargoTiltOne, true, 0.1);
    Robot.initVictor(CargoHelpOne, false, 0.1);

    // init some new values for elevator motor and overwrite some existing in initTalon
    CargoTiltOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    CargoTiltOne.setSelectedSensorPosition(0);
    CargoTiltOne.setSensorPhase(true);
    CargoTiltOne.configNominalOutputForward(0, 10);
    CargoTiltOne.configNominalOutputReverse(0, 10);
    CargoTiltOne.setNeutralMode(NeutralMode.Brake);
    dTiltControl = 0.0;
  }

  // ==== pneumatics methods/functions ====
  public void extendHatch(boolean bExtend) {
    hatchCylinder.set(bExtend);
  }

  public void liftFront(boolean bLift) {
    frontLift.set(bLift);
  }

  public void liftBack(boolean bLift) {
    backLift.set(bLift);
  }

  public void grabHelper(boolean bLift) {
    helperLift.set(bLift);
  }
  // ==== END of pneumatics methods/functions ====

  // ==== elevator methods/functions ===
  public void setElevatorPosition(int iPosition) {
    ElevatorOne.setSelectedSensorPosition(iPosition);
  }

  public int getElevatorPosition() {
    int iElevatorPositionTicks = ElevatorOne.getSelectedSensorPosition();
    return iElevatorPositionTicks;
  }

  public double gotoElevatorPosition(double dDesiredPosition, boolean bAuto, double dJoystickCommand) {
    // Note:  position should be set such that moving the elevator up results in a bigger number and zero is at the bottom
    dActualPosition = Robot.allsystems.getElevatorPosition();
    if (bAuto) {
      if (bElevMode) {
        dError = dDesiredPosition - dActualPosition;
        dDeriv = dActualPosition - dActualPositionOld;
        dControlDelta = dError / 40000.0 - dDeriv / 2500.0;
        dControlDelta = Math.signum(dControlDelta) * Math.min(Math.abs(dControlDelta), 0.05);
        dControl = dControlOld + dControlDelta;
        if (dActualPosition < dELEVATOR_TOP_POSITION && dControl < 0.0) {
          dControl = Math.min(0.1, Math.abs(dControl)) * Math.signum(dControl);
        }
        else if (dActualPosition > dELEVATOR_BOTTOM_POSITION && dControl > 0.0) {
          dControl = Math.min(0.2, Math.abs(dControl)) * Math.signum(dControl);
        }
      //  dControlLimited = Math.signum(dControl) * Math.min(Math.abs(dControl), 0.9);
      if (dControlLimited > 0.0) {
        dControlLimited = Math.signum(dControl) * Math.min(Math.abs(dControl), 0.7);
      }
      else {
        dControlLimited = Math.signum(dControl) * Math.min(Math.abs(dControl), 0.9);
      }
      ElevatorOne.set(ControlMode.PercentOutput, dControlLimited); 
        dControlOld = dControlLimited;
      }
      else {
        ElevatorOne.set(ControlMode.Position, dDesiredPosition);
      }
    }
    else {
      ElevatorOne.set(ControlMode.PercentOutput, dJoystickCommand);
      dControl = dJoystickCommand;
      dControlOld = 0.0;
    }
    dActualPositionOld = dActualPosition;
    return dControl;
  }

  public double getElevatorOutput() {
    return ElevatorOne.getMotorOutputPercent();
  }
  // ==== END of elevator methods/functions ===

  // ==== cargo methods/functions ===
  //    cargo manual functions
  public void setCargoGrabMotors(double Speed) {
    CargoGrabOne.set(ControlMode.PercentOutput, -Speed);
    CargoGrabTwo.set(ControlMode.PercentOutput, Speed);
  }

  public void setCargoTiltMotor(double Speed) {
    CargoTiltOne.set(ControlMode.PercentOutput, Speed);
  }

  public void setCargoHelpMotor(double Speed) {
    CargoHelpOne.set(ControlMode.PercentOutput, Speed);
  }

  //    cargo autotilt functions
  public void setTiltPosition(int iPosition) {
    CargoTiltOne.setSelectedSensorPosition(iPosition);
  }

  public int getTiltPosition() {
    int iTiltPositionTicks = - CargoTiltOne.getSelectedSensorPosition();
    return iTiltPositionTicks;
  }

  public double gotoTiltPosition(double dTiltDesiredPosition, boolean bAuto, double dJoystickCommand) {
    // Note:  position should be set such that moving the tilt up results in a bigger number and zero is at the bottom
    dTiltActualPosition = Robot.allsystems.getTiltPosition();
    if (bAuto) {
      dTiltError = dTiltDesiredPosition - dTiltActualPosition;
      dTiltDeriv = dTiltActualPosition - dTiltActualPositionOld;
      dTiltControlDelta = dTiltError / 40000.0 - dTiltDeriv / 2500.0;
      dTiltControlDelta = Math.signum(dTiltControlDelta) * Math.min(Math.abs(dTiltControlDelta), 0.03);
      dTiltControl = dTiltControlOld + dTiltControlDelta;
      if (dTiltActualPosition > dTILT_TOP_POSITION && dTiltControl > 0.0) {
        dTiltControl = Math.min(0.2, Math.abs(dTiltControl)) * Math.signum(dTiltControl);
      }
      else if (dTiltActualPosition < dTILT_BOTTOM_POSITION && dTiltControl < 0.0) {
        dTiltControl = Math.min(0.1, Math.abs(dTiltControl)) * Math.signum(dTiltControl);
      }
      dTiltControlLimited = Math.signum(dTiltControl) * Math.min(Math.abs(dTiltControl), 0.5);
      CargoTiltOne.set(ControlMode.PercentOutput, dTiltControlLimited);
      dTiltControlOld = dTiltControlLimited;
    }
    else {
      CargoTiltOne.set(ControlMode.PercentOutput, dJoystickCommand);
      dTiltControl = dJoystickCommand;
      dTiltControlOld = 0.0;
    }
    dTiltActualPositionOld = dTiltActualPosition;
    return dTiltControl;
  }

  public double getTiltOutput() {
    return CargoTiltOne.getMotorOutputPercent();
  }
  // ==== END of cargo methods/functions ===

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    CompressorOne.setClosedLoopControl(true);
    setDefaultCommand(new AllCommands());
  }
}
