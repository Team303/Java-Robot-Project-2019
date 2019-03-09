/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Timer;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import com.revrobotics.CANSparkMax;
import frc.robot.action.ActionWait;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {

  public static enum Position {LEFT, CENTER, RIGHT};
  public static enum Auto {DO_NOTHING, DRIVER_CONTROL, ONE_HATCH, TWO_HATCH, ROCKET, FORWARD};

  public static Drivebase drivebase;
  public static NavX navX;
  public static Autonomous auto;
  public static Lift lift;
  public static Compressor compressor;
  public static Intake intake;
  public static Camera camera;
  public static Climber climber;
  public static Timer timer;

  public static boolean rocketLift = false;
  public static int highSetpoint;
  public static int midSetpoint;
  public static int lowSetpoint;

  public int maxVel = 100;
  public int maxAccel = 100;
  public int maxJerk = 100;

  public static boolean autoControlA = true;
  public static boolean autoControlT = false;

  public static boolean userInputAuto = false;
  private SendableChooser<Position> positionChooser = new SendableChooser<>();
  private SendableChooser<Auto> autoChooser = new SendableChooser<>();


  @Override
  public void robotInit() {
    drivebase = new Drivebase();
    lift = new Lift();
    intake = new Intake();
    navX = new NavX();
    auto = new Autonomous();
    camera = new Camera();
    compressor = new Compressor();
    climber = new Climber();
    drivebase.zeroEncoders();
    timer = new Timer();
    SmartDashboard.putNumber("Time Push", 0.25);

    lift.zeroEncoders();
    navX.zeroYaw();    
    

    positionChooser.addOption("Left", Position.LEFT);
    positionChooser.addOption("Right", Position.RIGHT);
    positionChooser.addOption("Center", Position.CENTER);

    for (Auto auto: Auto.values()) {
      autoChooser.addOption(auto.toString(), auto);
    }

    SmartDashboard.putData("Position", positionChooser);
		SmartDashboard.putData("Auto", autoChooser);

  }

  @Override
  public void robotPeriodic() {

    OI.update();

    if (OI.leftZ < 0.5) {
      drivebase.zeroEncoders();
      navX.zeroYaw();
      lift.zeroEncoders();
      lift.setSetpoint(0);
      intake.zeroWristEncoder();
      intake.setWristSetpoint(0);
    }

    updateDashboard();
  }

  @Override
  public void autonomousInit() {

    compressor.setClosedLoopControl(false);
    compressor.stop();

    autoControlA = true;

    Position position = positionChooser.getSelected();

    if (position == Position.LEFT) {
      assembleLeftAutos();
    } else if (position == Position.RIGHT) {
      assembleRightAutos();
    } else if (position == Position.CENTER) {
      assembleCenterAutos();
    }

  }

  public void assembleLeftAutos() {
    Auto selected = autoChooser.getSelected();
    if (selected == Auto.DO_NOTHING) {}
    else if (selected == Auto.FORWARD) {auto.assembleForward();}
    else if (selected == Auto.DRIVER_CONTROL) {autoControlA = false;}
    else if (selected == Auto.ONE_HATCH) {auto.assembleRightOneHatch();}
    else if (selected == Auto.TWO_HATCH) {}
    else if (selected == Auto.ROCKET) {}

    auto.arr.add(new ActionWait(9999));

  }

  public void assembleCenterAutos() {
    Auto selected = autoChooser.getSelected();
    if (selected == Auto.DO_NOTHING) {}
    else if (selected == Auto.FORWARD) {auto.assembleForward();}
    else if (selected == Auto.DRIVER_CONTROL) {autoControlA = false;}
    else if (selected == Auto.ONE_HATCH) {auto.assembleCenterOneHatch();}
    else if (selected == Auto.TWO_HATCH) {}
    else if (selected == Auto.ROCKET) {}

    auto.arr.add(new ActionWait(9999));

  }

  public void assembleRightAutos() {
    Auto selected = autoChooser.getSelected();
    if (selected == Auto.DO_NOTHING) {}
    else if (selected == Auto.FORWARD) {auto.assembleForward();}
    else if (selected == Auto.DRIVER_CONTROL) {autoControlA = false;}
    else if (selected == Auto.ONE_HATCH) {}
    else if (selected == Auto.TWO_HATCH) {}
    else if (selected == Auto.ROCKET) {}  
    auto.arr.add(new ActionWait(9999));

  
  }

  @Override
  public void autonomousPeriodic() {
    
    updateDashboard();
    OI.update();

    if ((OI.xBtnA || OI.rTrigger) && autoControlA) {
      auto.stopAuto();
      autoControlA = false;
    } 

    if (autoControlA) {
      auto.run();
    } else {
      teleopPeriodic();
    }
  }

 @Override
  public void teleopInit() {
    
    super.teleopInit();
    compressor.setClosedLoopControl(true);
    autoControlT = false;
    auto.arr.clear();
    Robot.navX.zeroYaw();
    timer.reset();
    timer.start();
    Robot.lift.setSetpoint(0);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    OI.update();

    if (OI.lBtn[6] || OI.rBtn[6]) {
      autoControlT = true;
      if (OI.rBtn[6]) {
        Robot.camera.dumbVision = true;
      } else {
        Robot.camera.dumbVision = false;
      }
    } else {
      auto.teleopFirst = true;
      auto.taskNum = 100;
      autoControlT = false;
    }

    if (autoControlT) {
      auto.control();
    } else {
      teleopControl();
    }

  }

  public void teleopControl() {
    OI.update();
    updateDashboard();
    camera.updateVision();

    double driveAlter = (-Math.abs(1-((Robot.lift.getMainEncoder() + 230000)/225000.0))) + 1;
    double lDrivePower = (OI.lBtn[7]) ? -OI.leftY : -OI.leftY*driveAlter;
    double rDrivePower = (OI.lBtn[7]) ? -OI.rightY : -OI.rightY*driveAlter;

    climber.control();
    intake.control();
    lift.control();

    double adjustment = SmartDashboard.getNumber("Adjustment", 0.8);
    if ((Math.abs(OI.leftY) > 0.1) || (Math.abs(OI.rightY) > 0.1)) {
      drivebase.drive(adjustment * lDrivePower,  adjustment * rDrivePower);
    } else {
     drivebase.drive(0,0);
    }
  }


  public void updateDashboard() {
    SmartDashboard.putNumber("NavX Yaw", navX.getYaw());
    
    int direction = Robot.camera.visionDirection;
    String status = "";
    if (direction == 0) { status = "Not Set"; }
    else if (direction == 1) { status = "LEFT"; }
    else if (direction == 2) { status = "MIDDLE"; }
    else if (direction == 3) { status = "RIGHT"; }
    SmartDashboard.putString("Vision Direction", status);
    SmartDashboard.putNumber("Desired Heading", camera.getDesiredHeading(false));
    SmartDashboard.putNumber("Num Targets", camera.numValid);

    SmartDashboard.putNumber("Lift Encoder", Robot.lift.getMainEncoder());
    SmartDashboard.putNumber("Wrist Encoder", Robot.intake.getWristPosition());

    String autoStatus = "";
    if (autoControlA) {autoStatus = "AUTO";}
    else {status = "DRIVER CONTROL";}
    SmartDashboard.putString("Auto Status", autoStatus);

    if (autoControlT) {autoStatus = "AUTO";}
    else {autoStatus = "DRIVER CONTROL";}
    SmartDashboard.putString("Teleop Status", autoStatus);


  }


  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
