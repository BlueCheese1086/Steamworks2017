package org.usfirst.frc.team1086.robot.subsystems;

import org.usfirst.frc.team1086.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTBH {
	//An attempt to implement Take Back Half speed control
	
	public CANTalon shooter1, shooter2;
	private double targetRPM = 0;
	private double motorPower = 0;
	private static final double gain = 0.00001;
	private double lastError = 10;
	private double tbh = 0;
	public boolean isShooting;
	
    public ShooterTBH(){
        shooter1 = new CANTalon(RobotMap.SHOOTER1);
        shooter2 = new CANTalon(RobotMap.SHOOTER2);
        isShooting = false;
    }
    public double getRPM(){
    	return -shooter2.getEncVelocity() * 3.0 / 1024 * 60;
    }
    public void setRPM(int newRPM){
    	System.out.println("Setting RPM to: " + targetRPM); 
    	
    	this.targetRPM = newRPM;
    }
    public void outputData(){
    	SmartDashboard.putNumber("Shooter Speed", getRPM());
    	SmartDashboard.putNumber("Shooter Error", targetRPM - getRPM());
    }
    public void shoot(){
    	double error = this.targetRPM - getRPM();
    	motorPower += gain * error;
    	motorPower = clamp(motorPower);
    	if(isPositive(error) != isPositive(lastError)){
    		motorPower = 0.5 * (motorPower + tbh);
    		tbh = motorPower;
    		lastError = error;
    	}
    	System.out.println("Motor Power: " + motorPower);
    	shooter1.set(motorPower);
    	shooter2.set(motorPower);
    }
    public void stop(){
    	this.targetRPM = 0;
    	shooter1.set(0);
    	shooter2.set(0);
    	tbh = 0;
    	lastError  = 10;
    	isShooting = false;
    }
    public double clamp(double input){
    	if(input > 1) return 1.0;
    	if(input < -1) return -1.0;
    	return input;
    }
    public boolean isPositive(double input){
    	return input > 0;
    }
}
