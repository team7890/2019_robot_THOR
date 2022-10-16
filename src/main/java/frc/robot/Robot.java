/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.cscore.VideoSink;  // for camera switching


//wpi libraries
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.commands.AllCommands;

//:Commands
// import frc.robot.commands.ArcadeDrive;
// import frc.robot.commands.HandleCargo;
// import frc.robot.commands.HoldLift;
// import frc.robot.commands.LiftElevator;
// import frc.robot.commands.Pneumatics;

//:SubSystems
import frc.robot.subsystems.AllSystems;
import frc.robot.subsystems.Drivebase;

//ctre libraries
//import com.ctre.phoenix.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  //subsystem classes
  public static AllSystems allsystems;
  public static Drivebase drivebase;
  //command classes
  // public static ArcadeDrive arcadedrive;
  // public static HandleCargo handlecargo;
  // public static HoldLift holdlift;
  // public static LiftElevator liftelevator;
  // public static Pneumatics pneumatics;
  // // Root Classes
  public static OI oi;
  // Shuffleboard
  // private ShuffleboardTab shuffTab = Shuffleboard.getTab("Robot7890");
  // private NetworkTableEntry shuffMessage = shuffTab.add("Robot Mode", "none").withSize(2, 1).withPosition(0, 0).getEntry();
    
  UsbCamera camera1;
  UsbCamera camera2;
   
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    // init root classes
    oi = new OI();

    // init subsystems
    allsystems = new AllSystems();
    drivebase = new Drivebase();
    // init camera
    camera1 = CameraServer.startAutomaticCapture(0);
    camera2 = CameraServer.startAutomaticCapture(1);
    //camera1.setFPS(25);
    //camera1.setVideoMode(VideoMode.PixelFormat.kGray, 640, 480, 30);

    // init encoders
    Robot.allsystems.setElevatorPosition(0);
    Robot.allsystems.setTiltPosition(0);
    
    
    // shuffleboard message
    //shuffMessage.setString("Robot Init");
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // shuffleboard message
    //shuffMessage.setString("Robot Periodic");
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    Robot.allsystems.setElevatorPosition(0);
    // shuffleboard message
    //shuffMessage.setString("Disabled Init");
  }

  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
    // shuffleboard message
    //shuffMessage.setString("Disabled Periodic");
  }

  @Override
  public void autonomousInit() {
    // shuffleboard message
    //shuffMessage.setString("Auto Init");
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    // shuffleboard message
    //shuffMessage.setString("Auto Periodic");
  }

  @Override
  public void teleopInit() {
    // shuffleboard message
    //shuffMessage.setString("Teleop Init");
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
    // shuffleboard message
    //shuffMessage.setString("Teleop Periodic");
  }

  public static void initVictor(VictorSPX motor, boolean neutralBrake, double rampLimit) {
    if (neutralBrake) {
      motor.setNeutralMode(NeutralMode.Brake);
    }
    else {
      motor.setNeutralMode(NeutralMode.Coast);
    }
	 	motor.neutralOutput();
	 	motor.setSensorPhase(false);
	 	motor.configNominalOutputForward(0.0, 0);
    motor.configNominalOutputReverse(0.0, 0);
    motor.configOpenloopRamp(rampLimit, 50);
  }
  
  public static void initTalon(TalonSRX motor, boolean neutralBrake, double rampLimit) {
    if (neutralBrake) {
      motor.setNeutralMode(NeutralMode.Coast);
    }
    else {
      motor.setNeutralMode(NeutralMode.Brake);
    }
		motor.neutralOutput();
		//motor.setSensorPhase(false);
    motor.configOpenloopRamp(rampLimit, 50);
  }

  @Override
  public void testPeriodic() {
    Scheduler.getInstance().run();
    // shuffleboard message
    //shuffMessage.setString("Test Periodic");
  }
}
