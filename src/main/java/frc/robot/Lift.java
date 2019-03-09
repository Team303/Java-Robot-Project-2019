

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Lift {

    public CANSparkMax mainLift;
    public CANSparkMax followerLift;
    
    public CANEncoder mainEncoder;
    public CANEncoder followerEncoder;

    public double lastError;

    public CANPIDController mainPID;
    public CANPIDController followerPID;
    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

    private int liftSetpoint = 0;

    double setpoint = 0;

    public Lift() {
        mainLift = new CANSparkMax(RobotMap.MAIN_LIFT,  CANSparkMax.MotorType.kBrushless);
        followerLift = new CANSparkMax(RobotMap.FOLLOWER_LIFT,  CANSparkMax.MotorType.kBrushless);
        
        mainLift.setInverted(RobotMap.MAIN_LIFT_INV);
        followerLift.setInverted(RobotMap.FOLLOWER_LIFT_INV);

        mainPID = mainLift.getPIDController();
        followerPID = followerLift.getPIDController();
        
        mainEncoder = mainLift.getEncoder();
        followerEncoder = followerLift.getEncoder();

        mainEncoder.setPositionConversionFactor(1000);
        followerEncoder.setPositionConversionFactor(1000);
        
        zeroEncoders();
        setpoint = 0;
    }

    public void liftPID() {

        double d = 0;//.getNumber("Lift D", 1);
        double error = setpoint - getMainEncoder();
        double power = 0;
        
        if(error> 0) { //going up
			power = Math.pow(error/25000, 0.60);
        } else { //going down
			power = (error/65000);				
			power = Math.max(-0.5, power);
        }

        double derivative = (error - lastError) / 0.02;
        derivative = 0;
        setLiftPower(power + d*derivative);

        lastError = error;
    }

    //HELPER METHODS
    public void setLiftPower(double power) {
        mainLift.set(power);
        followerLift.set(power);
        
    }

    public void zeroEncoders() {
        mainLift.setEncPosition(0);
        followerLift.setEncPosition(0);
    }

    
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void control() {
        liftPID();
        
        if (OI.xlY < -0.3) { //
            setSetpoint(getSetpoint() + 400);
            //setLiftPower(0.5);
        } else if (OI.xlY > 0.3 && getSetpoint() >= -100){
            setSetpoint(getSetpoint() - 400);
            //setLiftPower(-0.25);
        } else {
            //setLiftPower(0);
        }
      
        
        int[] liftSetpoint = new int[3];
        if (OI.xLeftTrigger <= 0.75) liftSetpoint = new int[] {RobotMap.LOW_HATCH_SETPOINT, 
            RobotMap.MID_HATCH_SETPOINT, RobotMap.HIGH_HATCH_SETPOINT};
        else liftSetpoint = new int[] {RobotMap.LOW_ROCKET_SETPOINT, 
            RobotMap.MID_ROCKET_SETPOINT, RobotMap.HIGH_ROCKET_SETPOINT};

        if (OI.xPov == 0) { //Up
            setSetpoint(liftSetpoint[2]);
        } else if (OI.xPov == 180) { //Down
            setSetpoint(0);
        } else if (OI.xPov == 90) { //Right
            setSetpoint(liftSetpoint[1]);
        } else if (OI.xPov == 270) { //Left
            setSetpoint(liftSetpoint[0]);
        }

    }

    public double getMainEncoder() {
        return mainEncoder.getPosition();
    } 

    public double getFollowerEncoder() {
        return followerEncoder.getPosition() ;
    }

}

        //mainPID.setReference(setpoint, ControlType.kSmartMotion);
        //followerPID.setReference(setpoint, ControlType.kSmartMotion);

        //mainPID.setReference(setpoint, ControlType.kPosition);
        //followerPID.setReference(setpoint, ControlType.kPosition);
/* kP = SmartDashboard.getNumber("Lift P", 1);  
        kI = SmartDashboard.getNumber("Lift I", 0);
        kD = SmartDashboard.getNumber("Lift D", 0); 
        kIz = 0; 
        kFF = 0; 
        kMaxOutput = 1; 
        kMinOutput = -1;
    
        // set PID coefficients
        mainPID.setP(kP);
        mainPID.setI(kI);
        mainPID.setD(kD);
        mainPID.setOutputRange(kMinOutput, kMaxOutput);

        followerPID.setP(kP);
        followerPID.setI(kI);
        followerPID.setD(kD);
        followerPID.setOutputRange(kMinOutput, kMaxOutput);
        
        //ONLY FOR SMART MOTION
        //mainPID.setSmartMotionMaxVelocity(2000, 0);
        //mainPID.setSmartMotionMaxAccel(1500, 0);*/

