/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.Notifier;


/**
 * Add your docs here.
 */
public class Intake {

    private Solenoid deploy1;
    private Solenoid deploy2;

    private Solenoid extend1;
    private  Solenoid extend2;
    private  CANSparkMax wrist;
    private CANEncoder wristEncoder;

   // private PWMTalonSRX cargoIntake;
    private WPI_TalonSRX cargoIntake;
    private int wristSetpoint = 0;

    private boolean snipSnipped = false;
    private boolean started = false;
    public Notifier snipNotifier;

    private Timer snipTimer;

    public CANPIDController wristPID;

    public Intake() {
        //prac is 1,2,3,4
        //comp is 4,5
        deploy1 = new Solenoid(4);
        //deploy2 = new Solenoid(1);
    
        extend1 = new Solenoid(5);
        //extend2 = new Solenoid(3);

        wrist = new CANSparkMax(8, MotorType.kBrushless);
        //cargoIntake = new PWMTalonSRX(1);
        cargoIntake = new WPI_TalonSRX(11);
        cargoIntake.setInverted(true);

        

        snipTimer = new Timer();

        wristPID = wrist.getPIDController();
        wristEncoder = wrist.getEncoder();
        wristSetpoint = 0;
        wristEncoder.setPositionConversionFactor(1000);

        wristPID.setP(0.00004);
        wristPID.setI(0);
        wristPID.setD(0);

        zeroWristEncoder();
        deploy(false);
        extend(false);
        snipSnipped = false;
        started = false;

        snipNotifier = new Notifier(()->{
            if (!snipSnipped) {
        
                if (!started) {
                    snipTimer.reset();
                    snipTimer.start();
                    started = true;
                }
    
                extend(true);
    
                if (snipTimer.get() > 0.05 && started) {
                    deploy(true);
                    snipNotifier.stop();
                    snipSnipped = true;
                }
            }

        });
    }

    public void deploy(boolean state) {
        deploy1.set(state);
       // deploy2.set(state);
    }

    public void zeroWristEncoder() {
        wristEncoder.setPosition(0);
    }

    public void extend(boolean state) {
        extend1.set(state);
       // extend2.set(state);
    }

    public void setCargoIntake(double power) {
        //cargoIntake.set(power);
        cargoIntake.set(ControlMode.PercentOutput, power);
    }

    public void setWristPower(double power) {
        wrist.set(power);
    }

    public double getWristPosition() {
        return wristEncoder.getPosition();
    }

    public int getWristSetpoint() {
        return wristSetpoint;
    }

    public void wristPID() {
        wristPID.setReference(wristSetpoint, ControlType.kPosition); 
    }
    
    public void setWristSetpoint(int setpoint) {
        wristSetpoint = setpoint;
    }

    public void control() {
        wristPID();

        if (OI.lBtn[2]) {
            extend(true);
        } else if (OI.lBtn[5]){
            extend(false);
        }
 
        if (OI.rBtn[2] || OI.lBtn[1]) {
            deploy(true); 
        } else if (OI.rBtn[5] || OI.rBtn[1]) {
            deploy(false);
        }

        if (OI.lBtn[1]) {
            setCargoIntake(0.7);
        } else if (OI.rBtn[1]) {
            setCargoIntake(-0.7);    
        } else {
            setCargoIntake(0);
        }

        if (OI.xBtnA) {
            setWristSetpoint(RobotMap.WRIST_GROUND_SETPOINT);
        } else if (OI.xBtnY) {
            setWristSetpoint(RobotMap.WRIST_INITIAL_SETPOINT);
        } else if (OI.xBtnB || OI.xBtnY) {
            setWristSetpoint(RobotMap.WRIST_CARGO_SETPOINT);
        }

       if (OI.xrY < -0.3  &&  wristSetpoint >= -100) { //
            setWristSetpoint(wristSetpoint - 400);
        } else if (OI.xrY > 0.3){
            setWristSetpoint(wristSetpoint + 400);
        }

        SmartDashboard.putNumber("Intake Current", cargoIntake.getOutputCurrent());
        
        

    }


    public void checkIfIntakeCargo() {


        int threshold = 20;

        if (cargoIntake.getOutputCurrent() > threshold) {
            //Intake the ball
        }


    }

    public void snipSnip() {

        
        
        

    }


}