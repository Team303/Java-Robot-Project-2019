package frc.robot.action;

import frc.robot.Robot;

public class ActionWrist implements Action { 

    public int setpoint;

    public ActionWrist(int setpoint){
        this.setpoint = setpoint;
    }

    public void run() {
        Robot.intake.setWristSetpoint(setpoint);
    }

    public boolean isFinished() {
        return false;
    }    




}