package frc.robot.action;

import frc.robot.Robot;

public class ActionLift implements Action {
	private int setpoint;
	boolean firstRun = true;

	public ActionLift(int setpoint) {	
		this.setpoint = setpoint;
	}

	public void run() {
		if(firstRun) {
			Robot.lift.setSetpoint(setpoint);
			firstRun = false;
		}
		Robot.lift.liftPID();
	}

	public boolean isFinished() {
		return true;
	}


}