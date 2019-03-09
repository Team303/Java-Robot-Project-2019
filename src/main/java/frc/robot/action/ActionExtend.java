package frc.robot.action;

import frc.robot.Robot;

public class ActionExtend implements Action { 

    public boolean state;
    public ActionExtend(boolean state){
        this.state = state;
    }

    public void run() {
        Robot.intake.extend(state);
    }
    
    public boolean isFinished() {
        return true;
    }    
}