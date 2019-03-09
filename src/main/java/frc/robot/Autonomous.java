package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;
import frc.robot.action.ActionTrajectory;
import frc.robot.action.Action;
import frc.robot.action.ActionDeploy;
import frc.robot.action.ActionDriveStraightByEncoders;
import frc.robot.action.ActionWait;
import frc.robot.action.ActionDriveToGoalByWidth;
import frc.robot.action.ActionExtend;

public class Autonomous {

	ArrayList<Action> arr = new ArrayList<Action>();
	int taskNum = 0;
	boolean teleopFirst = true;
	boolean teleopRun = true;

	public void run() {
		if (arr.size() > taskNum) {
			arr.get(taskNum).run();
			if (arr.get(taskNum).isFinished()) {
				taskNum++;
			}
			teleopRun = false;
		} else {
			teleopRun = true;
		}

	}

	public void control() {
		//Run Once - Initialize the Run Array
		if (teleopFirst) {
			arr.clear();
			taskNum = 0;
			Robot.camera.visionDirection = 0;
			Robot.camera.turnConfirmed = false;
			driveToTarget(100, Robot.camera.getDesiredHeading(false));
			teleopFirst = false;
		}


		run();
	}

	public boolean getTeleopStatus() {
		return teleopRun;
	}

	public void stopAuto() {
		Robot.drivebase.drive(0,0);
		taskNum = 100;
	}

	public void assembleTest() {


		Waypoint[] testArr = {
			new Waypoint(8.5, 9.4, 0),
			new Waypoint(13.5, 5.5, 0),
			new Waypoint(18, 5.5, 0),
			new Waypoint(21.7, 7.9, Pathfinder.d2r(90))
		};

		//arr.add(new ActionTrajectory("Hatch1Left", 0,  0.01, false));
		//arr.add(new ActionDriveToGoalByWidth(125, 0));
		//launchPanel();
		//double inputPower=SmartDashboard.getNumber("Input Power", 0.6);
		//arr.add(new ActionDriveStraightByEncoders(28000, inputPower));
		//arr.add(new ActionDriveStraightByEncoders(28000, inputPower));
		arr.add(new ActionTrajectory("Straight", 0, 0.02, false));
		arr.add(new ActionWait(99999999));
	}


	public void assembleForward() {
		arr.add(new ActionDriveStraightByEncoders(30000, 0.65));
	}


	public void assembleCenterOneHatch() {
		arr.add(new ActionDriveStraightByEncoders(30000, 0.6));
		arr.add(new ActionDriveToGoalByWidth(100, 0, 1));
		launchPanel();
	}

	public void assembleRightOneHatch() {
		arr.add(new ActionTrajectory("Hatch1Right", 0, 0.01, false));
	}

	public void assembleTwoHatchLeft() {
		assembleOneHatchLeft();
	}

	public void assembleOneHatchLeft() {
		arr.add(new ActionTrajectory("Hatch1Left", 0, 0.01, false));
		scoreHatchPanel(250, 90);
	}


	public void scoreHatchPanel(int stopWidth, int desiredHeading) {
		driveToTarget(stopWidth, desiredHeading);
		launchPanel();
	}
	
	public void driveToTarget(int stopWidth, int desiredHeading) {
		arr.add(new ActionDriveToGoalByWidth(stopWidth, desiredHeading));
	}

	public void launchPanel() {
		arr.add(new ActionExtend(true));
		arr.add(new ActionDeploy(true));
		arr.add(new ActionWait(1.0));
		arr.add(new ActionExtend(false));
		arr.add(new ActionDeploy(false));
	}



}
