package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;

public class NavX implements PIDOutput { //this class controls the PID for the navX as well as the AHRS class itself
	AHRS navX;
	public PIDController2 turnController;
	double rate; //this is the output
	double setPoint = 0;
	double last_world_linear_accel_x;
	double last_world_linear_accel_y;
	static double kCollisionThreshold_DeltaG = 0.8f; 
	//static double kDefaultCollisionThreshold_DeltaG = 0.73f;
	
	double originalHeading = 0;

	public NavX() {
		navX = new AHRS(SPI.Port.kMXP);
		navX.setPIDSourceType(PIDSourceType.kDisplacement);

		//turnController = new PIDController(0.05, 0.01, 0.18, navX, this); //"kill it with the d" -Josh Tatum 2k17
		turnController = new PIDController2(0.4, 0.000, 0.000, navX, this, 30); //"kill it with the d" -Josh Tatum 2k17
		//0.65019
		//.2154
		turnController.setInputRange(-180.0f, 180.0f);
		turnController.setOutputRange(-0.55, 0.55);
		turnController.setAbsoluteTolerance(4.0f);
		turnController.setContinuous(true);
		originalHeading = 0;
	}

	public void setSetpoint(double setpoint) {
		setPoint = setpoint;
		turnController.setSetpoint(setpoint);
	}

	public double getSetpoint() {
		return setPoint;
	}

	public double getYaw() {
		return navX.getYaw();
	}
	public double getPitch() {
		return navX.getPitch();
	}
	public void zeroYaw() {
		//originalHeading = originalHeading + getYaw();
		navX.zeroYaw();
	}

	/*public double getOriginalYaw() {
		return originalHeading + getYaw();
	}
*/
	
	@Override
	public void pidWrite(double output) {		
		rate = output;
	}

	public double getPidOutput() {
		return rate; 
	}



}