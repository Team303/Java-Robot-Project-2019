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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

 
/**
 * Add your docs here.
 */
public class Climber {

    private Solenoid climberFront;
    private CANSparkMax climberBack;
    private boolean climberDown; 
    private CANEncoder backEncoder; 

    private static final double PITCH_CONSTANT = 0.01;


    
    private Timer deployTimer;
    private boolean deployed = false;

    public Climber() {
        climberFront = new Solenoid(RobotMap.CLIMBER_FRONT);
        climberBack = new CANSparkMax(RobotMap.CLIMBER_BACK, MotorType.kBrushless);
        deployTimer = new Timer();

        backEncoder = climberBack.getEncoder();

        backEncoder.setPositionConversionFactor(1000);
    }

    public void setFront(boolean state) {
        climberFront.set(state);
    }

    public void setBack(double power) {
        climberBack.set(power);
    }   


    public void zeroEncoders() {
        climberBack.setEncPosition(0);
    }



    public void control() {
       
        double power = SmartDashboard.getNumber("Back Climber Power", 0.5);
        double pitchConstant = SmartDashboard.getNumber("Climber Tuning Constant", 0.01);
        SmartDashboard.putNumber("Back Encoder", backEncoder.getPosition());


        double pitch = Robot.navX.getPitch() - 2;
    

        double powerAdjust = 0;

        if (Math.abs(pitch) >= 2.5) {
            powerAdjust = -pitch * pitchConstant;
        }

        SmartDashboard.putNumber("Power Adjust", powerAdjust);


        //-66906
        
        //If pitch is negative, decrease  power
            
        if (OI.xBtnBack) {
            setFront(false);
        }


        double stopPoint = -77200;
        double kP = SmartDashboard.getNumber("Back kP", 0.04);

        if (OI.lBtn[3]) {
            setBack(0.5);
        } else if (OI.lBtn[4]) {
            setBack(-0.5);
        } else if (OI.xBtnStart && backEncoder.getPosition() >= (stopPoint + 2500)) {
            setFront(true);
            setBack(-power + powerAdjust);
            System.out.println("Back Power: " +  climberBack.get());
        } else if (OI.xBtnStart && backEncoder.getPosition() <= (stopPoint + 2500)) {
            double error = stopPoint - backEncoder.getPosition();
            setBack(-error * kP);
            System.out.println("Back Power: " +  climberBack.get());
        } else {
            setBack(0);
        }


        SmartDashboard.putNumber("Back Power", -power + powerAdjust);



    }
}