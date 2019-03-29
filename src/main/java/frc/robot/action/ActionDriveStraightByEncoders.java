package frc.robot.action;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;

public class ActionDriveStraightByEncoders implements Action {

	public int distance = 0;
	public double power = 0.0;
	public double timeout = 0.0;
	public boolean firstRun = true;
	Timer timer = new Timer();
	public double initalYaw = 0.0;
	public int initial = 0;
	

	//165.2296//35.36 - 4.67 ft/s^2
	
	public ActionDriveStraightByEncoders(int distance, double power) {
		this(distance, power, 15);
	}

	public ActionDriveStraightByEncoders(int distance, double power, double timeout) {
		this.distance = distance;
		this.power = power;
		this.timeout = timeout; //in seconds
	}

	public void run() {
		if (firstRun) {
			initalYaw = Robot.navX.getYaw();
			timer.start();
			initial = Robot.drivebase.getLeftEncoder();
			firstRun = false;
		}

		//call drive straight - returns a double array with first index as left POWER, and second index as right POWER
		//driveStraight(power, angle difference, tuning constant)
		double[] pow = Action.driveStraight(power, Robot.navX.getYaw()-initalYaw, 0.005);
		Robot.drivebase.drive(pow[0], pow[1]);		
	}


	public boolean isFinished() {
		//Return true if the current encoder value is more or equal to the distance
		//OR return true if the time is more or equal to the timeout
		if(timer.get()>=timeout) timer.stop();

		boolean finished = Math.abs(Robot.drivebase.getLeftEncoder()-initial)>=Math.abs(distance) || timer.get() >=timeout;
		if (finished){
			Robot.drivebase.drive(0, 0);
		}
		
		return finished;
	}

}