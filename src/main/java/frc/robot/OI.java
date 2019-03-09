/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class OI {

    //Initialize Joysticks and XboxController
    public static Joystick left = new Joystick(0);
	public static Joystick right = new Joystick(1);
	public static XboxController xbox = new XboxController(2);
	public static boolean disabledState = false;
    
    //Left and Right X, Y, and Z axes
	public static double leftX = 0, leftY = 0, leftZ = 0;
	public static double rightX = 0, rightY = 0, rightZ = 0;
    
    public static double xlX = 0, xlY = 0, xrX = 0, xrY = 0;
    public static double xLeftTrigger=0, xRightTrigger=0;
	
	public static int xPov = 0;
	public static int lPov = 0;
	public static int rPov = 0;
	
	public static boolean[] lBtn = new boolean[9];
	public static boolean[] rBtn = new boolean[9];	
	public static boolean xBtnA, xBtnB, xBtnX, xBtnY, xLeftBumper, xRightBumper, xBtnStart, xBtnBack, xLeftStickBtn, xRightStickBtn, lTrigger, rTrigger, xStart, xSelect;
    
    //Called in Teleop Periodic (Loops every 2ms)

    public static void update() {
		for(int i=1;i<8;i++) { 
			lBtn[i] = left.getRawButton(i);
			rBtn[i] = right.getRawButton(i);
		}
		lPov = left.getPOV();
		rPov = right.getPOV();
		lTrigger = left.getTrigger();
		rTrigger = right.getTrigger();

		updateXboxValues();
	}
	
	public static void updateXboxValues() {
        
        //JOYSTICK: X,Y,Z axes
        leftX = left.getX();
		leftY = left.getY();
		leftZ = left.getZ();
				
		rightX = right.getX();
		rightY = right.getY();
		rightZ = right.getZ();
        
        
        //ALL XBOX BUTTONS/INPUTS
		xlX = xbox.getX(Hand.kLeft);
		xlY = xbox.getY(Hand.kLeft);
		xrX = xbox.getX(Hand.kRight);
		xrY = xbox.getY(Hand.kRight);
        
        //A,B,X,Y
		xBtnA = xbox.getAButton();
		xBtnB = xbox.getBButton();
		xBtnX = xbox.getXButton();
        xBtnY = xbox.getYButton();
        

        //Bumpers and Triggers
		xLeftBumper = xbox.getBumper(Hand.kLeft);
		xRightBumper = xbox.getBumper(Hand.kRight);
		xBtnStart = xbox.getStartButton();
		xBtnBack = xbox.getBackButton();
		xLeftStickBtn = xbox.getStickButton(Hand.kLeft);
		xRightStickBtn = xbox.getStickButton(Hand.kRight);
		xLeftTrigger = xbox.getTriggerAxis(Hand.kLeft);
		xRightTrigger = xbox.getTriggerAxis(Hand.kRight);
		xSelect = xbox.getBackButton();
		xStart = xbox.getStartButton();
		xPov = xbox.getPOV();
	}
	
	public static boolean pulse(boolean input){
		if(input){
			if(!disabledState){
				disabledState = true;
				return true;
			}
			return false;
		}
		disabledState = false;
		return false;
	}
	




}
