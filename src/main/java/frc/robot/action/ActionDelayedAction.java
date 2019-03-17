package frc.robot.action;

import edu.wpi.first.wpilibj.Timer;

public class ActionDelayedAction implements Action {

	Timer timer = new Timer();
	double time;
	boolean firstRun = true;
	Action action;

	/**
	 * Execute given action after a set delay. Useful for parallel actions where you don't want all the actions executing together.
	 * @param time the time to wait, in seconds
	 * @param action the action to run
	 */
	public ActionDelayedAction(double time, Action action) {
		this.time = time;
		this.action = action;
	}

	@Override
	public void run() {
		if(firstRun) {
			timer.start();
			firstRun = false;
		}

		if(timer.get()>=time) {
			action.run();
		};
	}

	@Override
	public boolean isFinished() {
		return action.isFinished();
	}

}