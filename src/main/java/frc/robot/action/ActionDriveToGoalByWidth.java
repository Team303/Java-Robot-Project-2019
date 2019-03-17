package frc.robot.action;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import static java.lang.Math.pow;

public class ActionDriveToGoalByWidth implements Action {
    boolean ranAngleChange;
    boolean goalFinished;
    boolean firstRun = true;
    int stopWidth;
    Timer timer;
    Timer countTimer;
    Timer turnTimer;
    boolean visionRun = false;
    static boolean reachedCenter = false;
    double distanceToCenter, desiredHeading, width, distance, initialHeading;
    int direction = -1;

    
    public ActionDriveToGoalByWidth(int stopWidth, double desiredHeading) {
        this(stopWidth, desiredHeading, -1);
    }


    public ActionDriveToGoalByWidth(int stopWidth, double desiredHeading, int direction) {
        firstRun = true;
        this.stopWidth = stopWidth;
        this.desiredHeading = desiredHeading;
        timer = new Timer();
        countTimer = new Timer();
        turnTimer = new Timer();
        this.direction = direction;
        firstRun = true;
    }
    
    @Override
    public void run() {

        if(firstRun) {
            timer.reset();
            timer.start();
            countTimer.reset();
            countTimer.start();
            turnTimer.reset();
            turnTimer.stop();
            
            reachedCenter = false;
            Robot.drivebase.zeroEncoders();            
            if (direction != -1) {
                Robot.camera.visionDirection = direction;
            }
            //Robot.camera.visionDirection = 0;

            firstRun = false;
        }

        SmartDashboard.putNumber("ABC Desired Heading",  desiredHeading);


        if (desiredHeading == 180) {
            Robot.camera.offsetConstant = 10;
        }

        Robot.camera.updateVision();
        double currentHeading = Robot.navX.getYaw();
        width = Robot.camera.getWidth();



        if (desiredHeading == 180 ) {
            if (currentHeading > 0) {
                currentHeading = currentHeading - 180;
            } else {
                currentHeading = 180 + currentHeading;
            }
            SmartDashboard.putNumber("Current Heading", currentHeading);
            desiredHeading = 0;
        }


        double differenceInHeading = currentHeading - desiredHeading;
        distanceToCenter = Robot.camera.getCentDist();
        double pixelOffset = Robot.camera.getPixelOffset(differenceInHeading);
        distanceToCenter += pixelOffset;

        //visionParameters = new double[] {19.778083727027587, -0.03188286910793101, 0.0007298420298416419, -0.00000215351397125946};
        //visionParameters = new double[] {17.593459373363583, -0.035055921410178395, 0.0004919578870525114, -9.320505240453085e-7};

        double[] visionParameters = new double[4];

        if (Math.abs(distanceToCenter) >= 170) {
            visionParameters = new double[] {18, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 127) {
            visionParameters = new double[] {18, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 100) {
            visionParameters = new double[] {16, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 70) {
            visionParameters = new double[] {15, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 45) {
            visionParameters = new double[] {12, 0, 0, 0};
        } else {
            visionParameters = new double[] {8, 0, 0, 0};
        }
        if (desiredHeading == 180) {
            System.out.println("DIST TO CENT: " + distanceToCenter);
            System.out.println("NUM VALID: " + Robot.camera.numValid);
            System.out.println("WIDTH: " + Robot.camera.getWidth());
        }


        double A = visionParameters[0];
        double B = visionParameters[1];
        double C = visionParameters[2];
        double D = visionParameters[3];
        double X = Math.abs(distanceToCenter);

        initialHeading = A + (B * X)  + (C * pow(X, 2)) + (D * pow(X, 3));
        initialHeading = Math.copySign(initialHeading, distanceToCenter);

        double threshold = 45;
        if ((Math.abs(distanceToCenter) <= threshold || Robot.camera.getWidth() >= 82) && !reachedCenter) {
            reachedCenter = true;
            turnTimer.reset();
            turnTimer.start();
        }




        double speed = 0.55;
        double angle = Robot.camera.getCameraDegreeOffset();

        double inputSpeed = 0.42;
        double inputSpeed2 = 0.46;
        double turningConstant = 0.005;

        if (!reachedCenter){
            speed = 0.45;
            angle = (desiredHeading + initialHeading) - currentHeading;
            turningConstant = 0.005;
        } else  {
            turningConstant = 0.0035;
            speed = inputSpeed2;
        }

        SmartDashboard.putBoolean("Reached Center", reachedCenter);
        SmartDashboard.putNumber("Width", width);
        SmartDashboard.putNumber("Distance To Center", distanceToCenter);
        SmartDashboard.putNumber("Difference In Heading", differenceInHeading);

        double[] pow = Action.driveStraight(speed, angle, turningConstant);
        Robot.drivebase.drive(pow[0], pow[1]);

        if (desiredHeading == 0 && Robot.camera.leftBtn) {
            desiredHeading = 180;
        }
    
    }

    @Override
    public boolean isFinished() {

        boolean finished = Robot.camera.getWidth() > stopWidth;
     
        return (Robot.camera.getWidth() > stopWidth);
    }   
    

}
