package frc.robot.action;

import frc.robot.Robot;

public class ActionIntakeSpin implements Action { 

    public double power;

    public ActionIntakeSpin(double power){
        this.power = power;
    }

    public void run() {
        Robot.intake.setCargoIntake(power);
    }
    
    public boolean isFinished() {
        return true;
    }    
    
}