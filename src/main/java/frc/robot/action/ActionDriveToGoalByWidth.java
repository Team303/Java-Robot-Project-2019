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
    private boolean loadingStation = false;

    private static final double A = -0.000042217484;
    private static final double B = 0.003231589;
    private static final double C = 0.58142857;


    
    public ActionDriveToGoalByWidth(int stopWidth, double desiredHeading) {
        this(stopWidth, desiredHeading, -1);
    }


    public ActionDriveToGoalByWidth(int stopWidth, double desiredHeading, int direction) {
        this(stopWidth, desiredHeading, direction, false);
    }

    public ActionDriveToGoalByWidth(int stopWidth, double desiredHeading, int direction, boolean loadingStation) {
        firstRun = true;
        this.stopWidth = stopWidth;
        this.desiredHeading = desiredHeading;
        timer = new Timer();
        countTimer = new Timer();
        turnTimer = new Timer();
        this.direction = direction;
        this.loadingStation = loadingStation;
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
        Robot.camera.lessOffset = false;

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
            visionParameters = new double[] {15, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 127) {
            visionParameters = new double[] {13, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 100) {
            visionParameters = new double[] {13, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 70) {
            visionParameters = new double[] {12, 0, 0, 0};
        } else if (Math.abs(distanceToCenter) >= 45) {
            visionParameters = new double[] {10, 0, 0, 0};
        } else {
            visionParameters = new double[] {8, 0, 0, 0};
        }
    


        double A = visionParameters[0];
        double B = visionParameters[1];
        double C = visionParameters[2];
        double D = visionParameters[3];
        double X = Math.abs(distanceToCenter);

        initialHeading = A + (B * X)  + (C * pow(X, 2)) + (D * pow(X, 3));
        initialHeading = Math.copySign(initialHeading, distanceToCenter);

        double threshold = 40;
        if ((Math.abs(distanceToCenter) <= threshold || Robot.camera.getWidth() >= 73) && !reachedCenter) {
            reachedCenter = true;
            turnTimer.reset();
            turnTimer.start();
        }




        double speed = 0.55;
        double angle = Robot.camera.getCameraDegreeOffset();

        double inputSpeed = 0.55;
        double inputSpeed2 = 0.6;
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
        SmartDashboard.putNumber("Angle", angle);

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


        double[] pow = Action.driveStraight(scaledPower, angle, turningConstant);
        Robot.drivebase.drive(pow[0], pow[1]);

        if (desiredHeading == 0 && (Robot.camera.leftBtn || loadingStation)) {
            desiredHeading = 180;
        }
    
    }

    @Override
    public boolean isFinished() {

        boolean finished = Robot.camera.getWidth() > stopWidth;
        return (Robot.camera.getWidth() > stopWidth);
    }   
    

}
