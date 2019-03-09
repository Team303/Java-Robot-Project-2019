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

/**
 * Add your docs here.
 */

 public class ActionTrajectory implements Action {

    private static final int k_ticks_per_rev = 8500;
    private static final double k_wheel_diameter = 0.5208333333;
    private static final double k_max_velocity = 10.0;  

    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;
  
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;
    private boolean isFirst = true;

    Notifier notifier;

    public static int counter = 0;
    public int offset = 0;
    public int leftEncoder = 0;
    public int rightEncoder = 0;

    public String trajectory;
    public double turningConstant;
    public boolean reversed;

    public ActionTrajectory(String trajectoryName, int offset, double turningConstant, boolean reversed) {
        trajectory = trajectoryName;
        this.offset = offset;
        this.turningConstant = turningConstant;
        this.reversed = reversed;


        notifier = new Notifier(()->{
            leftEncoder = Robot.drivebase.getLeftEncoder();
            rightEncoder = Robot.drivebase.getRightEncoder();
    
            if (reversed) {
                leftEncoder = -Robot.drivebase.getRightEncoder();
                rightEncoder = -Robot.drivebase.getLeftEncoder();
            }
        
            double leftSpeed = leftFollower.calculate(Robot.drivebase.getLeftEncoder());
            double rightSpeed = rightFollower.calculate(Robot.drivebase.getRightEncoder());
    
            if (reversed) {
                leftSpeed = -leftSpeed;
                rightSpeed = -rightSpeed;
            }
    
            double heading = Robot.navX.getYaw();
            double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
            double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
            double turn =  turningConstant * heading_difference;
    
            
            //turn = 0;        
            if (reversed) {
                Robot.drivebase.drive(rightSpeed + turn, leftSpeed - turn);
            } else {
                Robot.drivebase.drive(leftSpeed + turn, rightSpeed - turn);
            }

            if(!DriverStation.getInstance().isAutonomous() || DriverStation.getInstance().isDisabled()) {
				notifier.stop();
			}

        });
    }

    public void initTrajectories() { // dont be rowdy
        try {
        leftTrajectory = PathfinderFRC.getTrajectory("/output/" + trajectory + ".right");
        rightTrajectory = PathfinderFRC.getTrajectory("/output/" + trajectory + ".left");
        } catch (Exception e) {
            System.out.println(e);
        }
        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);
    
        leftFollower.configureEncoder(Robot.drivebase.getLeftEncoder(), k_ticks_per_rev, k_wheel_diameter);
        rightFollower.configureEncoder(Robot.drivebase.getRightEncoder(), k_ticks_per_rev, k_wheel_diameter);

        
        // You must tune the PID values on the following line!
        leftFollower.configurePIDVA(0.95, 0, 0, 1 / 12.0, 0);
        // You must tune the PID values on the following line!
        rightFollower.configurePIDVA(0.95,  0, 0, 1 / 12.0, 0);
    }

    public void run() {
        if (isFirst) {
            Robot.navX.zeroYaw();
            Robot.drivebase.zeroEncoders();
            initTrajectories();
            notifier.startPeriodic(0.02);
            isFirst = false;
        }
        
    }

    public boolean isFinished() {
        boolean finished = leftFollower.isFinished() && rightFollower.isFinished();
		if(finished) {
			notifier.stop();
		}
		return finished;
    }


}
