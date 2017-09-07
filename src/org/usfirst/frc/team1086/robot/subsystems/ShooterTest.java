package org.usfirst.frc.team1086.robot.subsystems;

import org.usfirst.frc.team1086.robot.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTest {
	public CANTalon shooter1, shooter2;
    //Solenoid flap = new Solenoid(RobotMap.FLAP);
    public ShooterTest(){
        shooter1 = new CANTalon(RobotMap.SHOOTER1);
        shooter2 = new CANTalon(RobotMap.SHOOTER2);
        
        shooter2.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
        shooter2.reverseSensor(true);
        shooter2.configNominalOutputVoltage(+0.0f, -0.0f);
        shooter2.configPeakOutputVoltage(+12.0f, -12.0f);
        
        shooter2.setProfile(0);
        shooter2.setP(0.02);
        shooter2.setI(0);
        shooter2.setD(0);
        shooter2.setF(0.028);
        
        shooter2.enableControl();
    }
    public double getRPM(){
    	return -shooter2.getEncVelocity() * 3.0 / 1024 * 60;
    }
    public void setRPM(int targetRPM){
    	
    }
    public void outputData(){
    	SmartDashboard.putNumber("Shooter Talon Speed", shooter2.getSpeed());
    	SmartDashboard.putNumber("Shooter Error", shooter2.getClosedLoopError() / 4096.0 * 600.0);
    	if(shooter2.getSetpoint() != 0)
    		System.out.println("Speed: " + shooter2.getSpeed() + ", Error: " + (shooter2.getClosedLoopError() / 4096.0 * 600.0) + ", Set: " + shooter2.getSetpoint());
    }
    public void shoot(){
    	shooter2.changeControlMode(TalonControlMode.Speed);
    	shooter2.set(-3500);
    }
    public void stop(){
    	//flap.set(false);
    	shooter1.set(0);
    	shooter2.set(0);
    }
}
