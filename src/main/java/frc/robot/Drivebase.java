/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxFrames.*;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Drivebase {    
    
    public CANSparkMax frontLeft;
    public CANSparkMax frontRight;
    public CANSparkMax backLeft;
    public CANSparkMax backRight;

    private CANEncoder leftEncoder;
    private CANEncoder rightEncoder;

    private CANPIDController leftPID;
    private CANPIDController rightPID;

    private SpeedControllerGroup leftMotors;
    private SpeedControllerGroup rightMotors;
    private DifferentialDrive drive;

     
    public Drivebase() {
        frontLeft = new CANSparkMax(RobotMap.FRONT_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        frontRight = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);
        backLeft = new CANSparkMax(RobotMap.BACK_LEFT, CANSparkMaxLowLevel.MotorType.kBrushless);
        backRight = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
    
        frontLeft.setInverted(RobotMap.FRONT_LEFT_INV);
        frontRight.setInverted(RobotMap.FRONT_RIGHT_INV);
        backLeft.setInverted(RobotMap.BACK_LEFT_INV);
        backRight.setInverted(RobotMap.BACK_RIGHT_INV);
       
        leftEncoder = new CANEncoder(frontLeft);
        rightEncoder = new CANEncoder(frontRight);
       
        rightMotors = new SpeedControllerGroup(backRight, frontRight);
        leftMotors = new SpeedControllerGroup(backLeft, frontLeft);

        drive = new DifferentialDrive(leftMotors, rightMotors);
        drive.setSafetyEnabled(false);
        
        leftEncoder.setPositionConversionFactor(1000);
        rightEncoder.setPositionConversionFactor(1000);
        zeroEncoders();

    }

    public void drive(double left, double right) {
        drive.tankDrive(-left, right);
       // frontLeft.set(left);
      //  backLeft.set(left);
       // frontRight.set(left);
       // frontRight.set(left);

    }

    public double getLeftSpeed() {
        return frontLeft.get();
    }

    public double getRightSpeed() {
        return frontRight.get();
    }

    public int getLeftEncoder() {
        return (int) -leftEncoder.getPosition();
    }

    public int getRightEncoder() {
        return (int) -rightEncoder.getPosition();
    }

    public void zeroEncoders() {
        frontLeft.setEncPosition(0);
        frontRight.setEncPosition(0);
    }

}
