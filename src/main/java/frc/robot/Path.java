/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import jaci.pathfinder.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Waypoint;
import java.util.HashMap;
import jaci.pathfinder.followers.EncoderFollower;

/**
 * Add your docs here.
 */

public class Path {

    private static final int k_ticks_per_rev = 8500;
    private static final double k_wheel_diameter = 0.5208333333;
    private static final double k_max_velocity = 10.0;  
    
    public HashMap<String, EncoderFollower[]> trajectoryMap;

    public Path() {
        trajectoryMap = new HashMap<>();

    }

    public HashMap<String, EncoderFollower[]> getTrajectoryMap() {
        return trajectoryMap;
    }

    public void initTrajectories(String[] trajectoryNames) {
        SmartDashboard.putBoolean("Trajectory Status", false);
        for (String trajectory : trajectoryNames) {
            System.out.println("TRAJECTORY: " + trajectory);
            trajectoryMap.put(trajectory, createTrajectory(trajectory));
        }
        SmartDashboard.putBoolean("Trajectory Status", true);
    }

    public EncoderFollower[] createTrajectory(String trajectory) { // dont be rowdy
       
        Trajectory leftTrajectory;
        Trajectory rightTrajectory;

            try {
            leftTrajectory = PathfinderFRC.getTrajectory("/output/" + trajectory + ".right");
            rightTrajectory = PathfinderFRC.getTrajectory("/output/" + trajectory + ".left");
            } catch (Exception e) {
                System.out.println(e);
                leftTrajectory = null;
                rightTrajectory = null;
            }
            EncoderFollower leftFollower = new EncoderFollower(leftTrajectory);
            EncoderFollower rightFollower = new EncoderFollower(rightTrajectory);

            System.out.println("DONE FOLLOWER");
    
            leftFollower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);
            rightFollower.configureEncoder(0, k_ticks_per_rev, k_wheel_diameter);
    
            double p = SmartDashboard.getNumber("NavX P", 0.95);
            leftFollower.configurePIDVA(p, 0, 0, 1 / 10.0, 0);
            rightFollower.configurePIDVA(p,  0, 0, 1 / 10.0, 0);
    

            System.out.println("DONE SECOND FOLLOWER");

            //notifier.startPeriodic(0.02);


            return new EncoderFollower[] {leftFollower, rightFollower};
    }



}