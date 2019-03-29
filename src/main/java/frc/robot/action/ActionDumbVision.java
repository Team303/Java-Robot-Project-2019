package frc.robot.action;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import static java.lang.Math.pow;

public class ActionDumbVision implements Action {
    boolean ranAngleChange;
    boolean goalFinished;
    boolean firstRun = true;
    int stopWidth;

    boolean visionRun = false;
    static boolean reachedCenter = false;
    double distanceToCenter, desiredHeading, width, distance, initialHeading;
    int direction = -1;
    private boolean loadingStation = false;

    private static final double A = -0.000042217484;
    private static final double B = 0.003231589;
    private static final double C = 0.58142857;


    
    public ActionDumbVision(int stopWidth, double desiredHeading) {
        this.stopWidth = stopWidth;
    }
    
    @Override
    public void run() {
   
        Robot.camera.lessOffset = true;
        Robot.camera.updateVision();

        double scaledPower = (A * pow(width, 2)) + (B * width) + C;

        scaledPower = 0.5;

        if (Math.abs(width) <= 60) {
            scaledPower = 0.55;
        } else if (Math.abs(width) <= 70) {
            scaledPower = 0.55;
        } else if (Math.abs(width) <= 80) {
            scaledPower = 0.5;
        } else if (Math.abs(width) <= 90) {
            scaledPower = 0.46;
        } else if (Math.abs(width) <= 100) {
            scaledPower = 0.43;
        } else if (Math.abs(width) <= 110) {
            scaledPower = 0.4;
        }else if (Math.abs(width) <= 120) {
            scaledPower = 0.4;
        }else {
            scaledPower = 0.4;
        }

        double dumbAngle = Robot.camera.getCameraDegreeOffset();
        double turningConstant = 0.004;


        double[] pow = Action.driveStraight(scaledPower, dumbAngle, turningConstant);
        Robot.drivebase.drive(pow[0], pow[1]);
    }

    @Override
    public boolean isFinished() {

        boolean finished = Robot.camera.getWidth() > stopWidth;
        return (Robot.camera.getWidth() > stopWidth);
    }   
    

}
