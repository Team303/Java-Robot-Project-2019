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
import frc.robot.action.ActionLift;
import frc.robot.action.ActionParallelAction;
import frc.robot.action.ActionTurnToAngle;
import frc.robot.action.ActionDelayedAction;
import frc.robot.action.ActionTrajectoryFirst;
import frc.robot.action.ActionDumbVision;

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
			driveToTarget(125, Robot.camera.getDesiredHeading(false));
			teleopFirst = false;
		}
		
		SmartDashboard.putBoolean("STOP VISION", (Robot.camera.getWidth() > 200));



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
		///arr.add(new ActionTrajectory("Straight", 0, 0.02, false));
		//arr.add(new ActionWait(99999999));
		arr.add(new ActionWait(99999999));

		
	}


	public void assembleForward() {
	}



	public void assembleCenterOneHatch() {
		/*arr.add(new ActionDriveStraightByEncoders(30000, 0.5));
		arr.add(new ActionDriveToGoalByWidth(130, 0, 1));
		arr.add(new ActionDeploy(true));*/
		arr.add(new ActionTurnToAngle(180, false, 10.0f));
		arr.add(new ActionParallelAction(new ActionWait(1.0), new ActionLift(3000)));
		arr.add(new ActionWait(99999));

	}

	public void assembleRightOneHatch() {
		arr.add(new ActionTrajectory("Hatch1Right", 0, 0.01, false));
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
		arr.add(new ActionWait(13.0));
		arr.add(new ActionExtend(false));
		arr.add(new ActionDeploy(false));
	}


	
	public void assembleOneRocketHatchLeft() {
		arr.add(new ActionDriveStraightByEncoders(12000, 0.45));
		//arr.add(new ActionWait(0.1));
		arr.add(new ActionTrajectory("Straight", 0, 0, false));
		arr.add(new ActionDriveToGoalByWidth(120, -20.5, 1));
		arr.add(new ActionTurnToAngle(-20, false, 5.0f));
		arr.add(new ActionWait(0.5));
		arr.add(new ActionExtend(true));
		arr.add(new ActionDeploy(false));
		arr.add(new ActionWait(99999));

	}


	public void assembleTwoRocketHatchLeft() {
		arr.add(new ActionWait(0.5)); //21000
		arr.add(new ActionDriveStraightByEncoders(10000, 0.45)); 
		arr.add(new ActionWait(0.1));
		arr.add(new ActionTrajectory("StraightLeft", 0, 0, false, 2.9));
		//arr.add(new ActionWait(1.0));
		arr.add(new ActionDumbVision(100, -30.0));
		arr.add(new ActionWait(0.5));
		arr.add(new ActionParallelAction(new ActionWait(0.5), new ActionDeploy(true)));

		//arr.add(new ActionParallelAction(new ActionDriveToGoalByWidth(150, -22.5, 1), new ActionLift(2000)));
		//arr.add(new ActionTurnToAngle(-30, false, 5.0f));
		arr.add(new ActionTurnToAngle(-18, false, 5.0f));
		arr.add(new ActionTrajectory("Back", 0, 0, true, 2.0));
		arr.add(new ActionParallelAction(new ActionWait(0.5), new ActionDeploy(false)));
		arr.add(new ActionTurnToAngle(180, false, 10.0f));
		arr.add(new ActionDriveToGoalByWidth(135, 180, 1, true));
		/*arr.add(new ActionParallelAction(new ActionWait(0.3), new ActionDeploy(true)));
		arr.add(new ActionParallelAction(new ActionTrajectory("Return", 0, 0, true), new ActionDelayedAction(1.0, new ActionLift(0))));
		arr.add(new ActionDriveToGoalByWidth(115, -153.5, 3));*/
		//arr.add(new ActionDeploy(true));

		//arr.add(new ActionTurnToAngle(-45, false, 5.0f));
		
		arr.add(new ActionWait(99999));
	}


	public void assembleTwoRocketHatchRight() {
		arr.add(new ActionWait(0.5)); //21000
		arr.add(new ActionDriveStraightByEncoders(10000, 0.45)); 
		arr.add(new ActionWait(0.1));
		arr.add(new ActionTrajectory("StraightRightNew", 0, 0, false, 2.9));
		//arr.add(new ActionWait(1.0));
		arr.add(new ActionDumbVision(100, -30.0));
		arr.add(new ActionWait(0.5));
		arr.add(new ActionParallelAction(new ActionWait(0.5), new ActionDeploy(true)));
		arr.add(new ActionTurnToAngle(18, false, 5.0f));
		arr.add(new ActionTrajectory("BackRight", 0, 0, true, 2.0));
		arr.add(new ActionTurnToAngle(180, false, 10.0f));
		arr.add(new ActionDriveToGoalByWidth(160, 180, 1, true));
		arr.add(new ActionParallelAction(new ActionWait(0.3), new ActionDeploy(true)));
		arr.add(new ActionParallelAction(new ActionTrajectory("ReturnTwo", 0, 0, true), new ActionDelayedAction(1.0, new ActionLift(0))));
		arr.add(new ActionDriveToGoalByWidth(115, 153.5, 3));
		//arr.add(new ActionDeploy(true));

		//arr.add(new ActionTurnToAngle(-45, false, 5.0f));

		arr.add(new ActionWait(99999));
	}


}
