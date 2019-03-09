package frc.robot.action;

import frc.robot.Robot;

public class ActionDeploy implements Action { 

    public boolean state;
    public ActionDeploy(boolean state){
        this.state = state;
    }

    public void run() {
        Robot.intake.deploy(state);
    }
    
    public boolean isFinished() {
        return true;
    }    
}