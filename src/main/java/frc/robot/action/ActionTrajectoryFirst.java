/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.action;

import frc.robot.Robot;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import edu.wpi.first.wpilibj.Notifier;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */

 public class ActionTrajectoryFirst implements Action {

    private static final int k_ticks_per_rev = 8500;
    private static final double k_wheel_diameter = 0.5208333333;
    private static final double k_max_velocity = 10.0;  

    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;
  
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;
    private boolean isFirst = true;

    private Timer trajectoryTimer;

    Notifier notifier;

    public int counter = 0;
    public int offset = 0;
    public int leftEncoder = 0;
    public int rightEncoder = 0;
    public int initialLeftEncoder = 0;
    public int initialRightEncoder = 0;

    public String trajectory;
    public double turningConstant;
    public boolean reversed;
    public boolean firstEncoder = false;

    public double stopTime;

    public ActionTrajectoryFirst(String trajectoryName, int offset, double turningConstant, boolean reversed) {
            this(trajectoryName, offset, turningConstant, reversed, 5.0);

        /*notifier = new Notifier(()->{
            /*leftEncoder = Robot.drivebase.getLeftEncoder();
            rightEncoder = Robot.drivebase.getRightEncoder();
    
            if (reversed) {
                leftEncoder = -Robot.drivebase.getRightEncoder();
                rightEncoder = -Robot.drivebase.getLeftEncoder();
            }

            EncoderFollower leftFollower = Robot.path.trajectoryMap.get(trajectoryName)[0];
            EncoderFollower rightFollower = Robot.path.trajectoryMap.get(trajectoryName)[];

            double leftSpeed = leftFollower.calculate(leftEncoder);
            double rightSpeed = rightFollower.calculate(rightEncoder);
    
            if (reversed) {
                leftSpeed = -leftSpeed;
                rightSpeed = -rightSpeed;
            }
    
            double heading = Robot.navX.getYaw();
            double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
            double turn =  turningConstant * heading_difference;
            turn = 0;

            double left = leftSpeed + turn;
            double right = rightSpeed - turn;
            left = 0.7 * left;
            right = 0.7 * right;

            System.out.println("Left: " + left);
            System.out.println("Left Encoder: " + leftEncoder);
            System.out.println("Right: " + right);
            System.out.println("Right Encoder: " + rightEncoder);
            //turn = 0;        
            if (reversed) {
                Robot.drivebase.drive(right, left);
            } else {
                Robot.drivebase.drive(left, right);
            }

            if(!DriverStation.getInstance().isAutonomous() || DriverStation.getInstance().isDisabled()) {
				notifier.stop();
            }
            
            boolean finished2 = leftFollower.isFinished() && rightFollower.isFinished();
            if(finished2) {
				notifier.stop();
            }

        });*/
    }

    public ActionTrajectoryFirst(String trajectoryName, int offset, double turningConstant, boolean reversed, double stopTime) {
        trajectory = trajectoryName;
        this.offset = offset;
        this.turningConstant = turningConstant;
        this.reversed = reversed;
        trajectoryTimer = new Timer();
        this.stopTime = stopTime;
        this.firstEncoder = false;
    
    }

    /*

    public void initTrajectories() { // dont be rowdy
        try {
        leftTrajectory = PathfinderFRC.getTrajectory("/output/" + trajectory + ".right");
        rightTrajectory = PathfinderFRC.getTrajectory("/output/" + trajectory + ".left");
        } catch (Exception e) {
            System.out.println(e);
        }
        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);
    
        SmartDashboard.putNumber("Initial Left", Robot.drivebase.getLeftEncoder());
        SmartDashboard.putNumber("Initial Right", Robot.drivebase.getRightEncoder());

        leftFollower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);
        rightFollower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);

        double p = SmartDashboard.getNumber("NavX P", 0.95);
        leftFollower.configurePIDVA(p, 0, 0, 1 / 10.0, 0);
        rightFollower.configurePIDVA(p,  0, 0, 1 / 10.0, 0);

        //notifier.startPeriodic(0.02);

    }*/

    public void run() {
        if (isFirst) {

            leftFollower = Robot.path.trajectoryMap.get(trajectory)[0];
            rightFollower = Robot.path.trajectoryMap.get(trajectory)[1];
            trajectoryTimer.reset();
            trajectoryTimer.start();
            System.out.println("STARTED");
            isFirst = false;
        }

            leftEncoder = Robot.drivebase.getLeftEncoder();
            rightEncoder = Robot.drivebase.getRightEncoder();

            if (!firstEncoder && trajectoryTimer.get() >= 0.3) {
                System.out.println("ZERO");
                
                initialLeftEncoder = leftEncoder;
                initialRightEncoder = rightEncoder;
                System.out.println("Left: " + initialLeftEncoder);
                System.out.println("Right: " + initialRightEncoder);
                firstEncoder = true;
            }
    
            if (reversed) {
                leftEncoder = -Robot.drivebase.getRightEncoder();
                rightEncoder = -Robot.drivebase.getLeftEncoder();
            }

            leftEncoder = leftEncoder - initialLeftEncoder;
            rightEncoder = rightEncoder - initialRightEncoder;

            double leftSpeed = leftFollower.calculate(leftEncoder);
            double rightSpeed = rightFollower.calculate(rightEncoder);
    
            if (reversed) {
                leftSpeed = -leftSpeed;
                rightSpeed = -rightSpeed;
            }
    
            double heading = Robot.navX.getYaw();
            double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
            double turn =  turningConstant * heading_difference;
            turn = 0;

            double left = leftSpeed + turn;
            double right = rightSpeed - turn;
            left = 0.7 * left;
            right = 0.7 * right;

            System.out.println(left + "    ||    " + right);
            System.out.println("Actual: " +  Robot.drivebase.getLeftEncoder());
            System.out.println("Adjusted: " + leftEncoder);
            System.out.println("Time: " + trajectoryTimer.get());

            //turn = 0;        
            if (reversed) {
                Robot.drivebase.drive(right, left);
            } else if (trajectoryTimer.get() >= 0.3){
                Robot.drivebase.drive(left, right);
            }
        
    }

    public boolean isFinished() {
        boolean finished = leftFollower.isFinished() && rightFollower.isFinished();
		boolean finished2 = (trajectoryTimer.get() >= stopTime) || finished;
        
        return finished2;
    }


}
