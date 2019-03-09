/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * What if i say no? BEN WUZ HERE 1/15/19
 */
public class RobotMap {

    public static final int FRONT_LEFT = 3;
    public static final int FRONT_RIGHT = 5;
    public static final int BACK_LEFT = 2;
    public static final int BACK_RIGHT = 4;

    public static final boolean FRONT_LEFT_INV = true;
    public static final boolean FRONT_RIGHT_INV = false;
    public static final boolean BACK_LEFT_INV = true;
    public static final boolean BACK_RIGHT_INV = false;

    public static final int MAIN_LIFT = 6;
    public static final int FOLLOWER_LIFT = 7;

    public static final boolean MAIN_LIFT_INV = true;
    public static final boolean FOLLOWER_LIFT_INV = false;

    public static final int WRIST = 8;
    public static final int CARGO_INTAKE = 9;

    public static final boolean WRIST_INV = false;
    public static final boolean CARGO_INTAKE_INV = false;

    //Solenoids
    public static final int DEPLOY_1 = 0;
    public static final int DEPLOY_2 = 1;
    public static final int EXTEND_1 = 2;
    public static final int EXTEND_2 = 3;

    //Don't know the ID's of this yet
    public static final int CLIMBER_FRONT = 7;
    public static final int CLIMBER_BACK = 6;
    public static final int CLIMBER_TRACTOR = 9;
    public static final boolean CLIMBER_TRACTOR_INV = false;

    public static final int LOW_ROCKET_SETPOINT = 14000;
    public static final int MID_ROCKET_SETPOINT = 47261;
    public static final int HIGH_ROCKET_SETPOINT = 74311;

    public static final int LOW_HATCH_SETPOINT = 0;
    public static final int MID_HATCH_SETPOINT = 34094;
    public static final int HIGH_HATCH_SETPOINT = 64406;

    public static final int WRIST_GROUND_SETPOINT = 32085; //Dont Know Yet
    public static final int WRIST_CARGO_SETPOINT = 22204; //Dont Know Yet
    public static final int WRIST_INITIAL_SETPOINT = 0;
    public static final int WRIST_SHIP_SETPOINT = 11881;

}
