/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.action;

import frc.robot.Robot;
import jaci.pathfinder.Waypoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.*;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;


/**
 * Add your docs here.
 */
public class ActionTrajectoryByWaypoint implements Action {

    public Waypoint[] waypointArr;

    private static final int k_ticks_per_rev = 8500;
    private static final double k_wheel_diameter = 0.5;
    private static final double k_max_velocity = 7;  
    
    private static final double wheelBaseWidth = 2.1667; 

    public static final double timeStep = 0.02;
    public static final double maxVel = 7.0;
    public static final double maxAccel = 6.0;
    public static final double maxJerk = 50.0;

    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;
  
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;
    private boolean isFirst = true;

    public static int counter = 0;
    public int offset = 0;
    public int leftEncoder = 0;
    public int rightEncoder = 0;

    public String trajectory;
    public double turningConstant = 0.01;
    public boolean reversed;

    public ActionTrajectoryByWaypoint(Waypoint[] waypointArr) {
        this.waypointArr = waypointArr;
    }

    public void initTrajectories() {

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
        timeStep, maxVel, maxAccel, maxJerk);

        Trajectory visionTrajectory = Pathfinder.generate(waypointArr, config);
        TankModifier testModifier = new TankModifier(visionTrajectory).modify(wheelBaseWidth);

        leftTrajectory = testModifier.getRightTrajectory();
        rightTrajectory = testModifier.getLeftTrajectory();

        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);
    
        leftFollower.configureEncoder(Robot.drivebase.getLeftEncoder(), k_ticks_per_rev, k_wheel_diameter);
        rightFollower.configureEncoder(Robot.drivebase.getRightEncoder(), k_ticks_per_rev, k_wheel_diameter);

        leftFollower.configurePIDVA(SmartDashboard.getNumber("P", 1.0), 0, 0, 1 / k_max_velocity, 0);
        rightFollower.configurePIDVA(SmartDashboard.getNumber("P", 1.0), 0, 0, 1 / k_max_velocity, 0);

    }

    public void run () {

        if (isFirst) {
            Robot.navX.zeroYaw();
            Robot.drivebase.zeroEncoders();
            initTrajectories();
            isFirst = false;
        } 

        double leftSpeed = leftFollower.calculate(Robot.drivebase.getLeftEncoder());
        double rightSpeed = rightFollower.calculate(Robot.drivebase.getRightEncoder());

        double heading = Robot.navX.getYaw();
        double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
        //System.out.println("LEFT: " + leftSpeed);
        //System.out.println("RIGHT: " + rightSpeed);
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        System.out.println("HEADING: " + heading);

        double turn =  turningConstant * heading_difference;
        
        Robot.drivebase.drive(leftSpeed + turn, rightSpeed -  turn);
    }

    public boolean isFinished() {
        return (leftFollower.isFinished() || rightFollower.isFinished());
    }

}
