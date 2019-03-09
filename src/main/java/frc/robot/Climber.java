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
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Add your docs here.
 */
public class Climber {

    private Solenoid climberFront;
    private Solenoid climberBack;
    private boolean climberDown; 
    private WPI_TalonSRX tractor;
    private Timer deployTimer;
    private boolean deployed = false;

    public Climber() {
        climberFront = new Solenoid(RobotMap.CLIMBER_FRONT);
        climberBack = new Solenoid(RobotMap.CLIMBER_BACK);

        tractor = new WPI_TalonSRX(RobotMap.CLIMBER_TRACTOR);
        tractor.setInverted(RobotMap.CLIMBER_TRACTOR_INV);

        deployTimer = new Timer();
    }

    public void setFront(boolean state) {
        climberFront.set(state);
    }

    public void setBack(boolean state) {
        climberBack.set(state);
    }   

    public void setTractor(double power) {
        tractor.set(ControlMode.PercentOutput, power);
    }

    public void control() {
       
        System.out.println("OI: " +  OI.xRightTrigger);

        if (OI.xRightTrigger >= 0.75) {
            climberDown = true;
            if (OI.xBtnStart) {
                if (!deployed) {
                    setFront(true);
                    deployTimer.reset();
                    deployTimer.start();
                    deployed = true;
                    SmartDashboard.putBoolean("Time On", false);
                }

                if (deployTimer.get() > SmartDashboard.getNumber("Time Push", 0.25) && deployed == true) {
                    SmartDashboard.putBoolean("Time On", true);
                    setBack(true);
                }
            } 
            
            if (OI.xLeftBumper) {
                setFront(false);
                setBack(true);
                deployed = false;
            }

            if (OI.xRightBumper) {
                setFront(false);
                setBack(false);
            }
        } else {
            climberDown = false;
        }
        

        if(climberDown) {
            setTractor(OI.leftY);
        } else {
            setTractor(0);
        }

        if (OI.lBtn[2]) {
            setBack(false);
        } 

        if (OI.lBtn[3]) {
            setBack(false);
        } 

    }
}