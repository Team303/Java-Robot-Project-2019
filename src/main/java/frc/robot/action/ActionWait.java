package frc.robot.action;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class ActionWait  implements Action{

	boolean firstRun;
	Timer t;
	double timeThreshold; //in seconds
	
	public ActionWait(double time) {
		timeThreshold = time;
		firstRun = true;
		t = new Timer();
		t.stop();
	}
	
	@Override
	public void run() {
		if(firstRun) {
			firstRun = false;
			t.start();
        }
        //Robot.drivebase.zeroEncoders();
        //Robot.drivebase.drive(0,0);
	}

	@Override
	public boolean isFinished() {
		boolean end = t.get()>=timeThreshold;
		
		if(end) {
			firstRun = true;
			t.stop();
			t.reset();
		}
		
		return end;
	}
	
}
