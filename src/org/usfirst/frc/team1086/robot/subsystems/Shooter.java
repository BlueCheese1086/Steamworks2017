
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc.team1086.robot.RobotMap;

public class Shooter implements PIDSource, PIDOutput {
    public CANTalon shooter1, shooter2;
    PIDController controller = new PIDController(0.00055, 0.00002, 0.0007, 0.022, this, this);
    public boolean isShooting = false;
    public double pidOutput = 0;
    //Solenoid flap = new Solenoid(RobotMap.FLAP);
    public Shooter(){
        shooter1 = new CANTalon(RobotMap.SHOOTER1);
        shooter2 = new CANTalon(RobotMap.SHOOTER2);
        shooter2.setEncPosition(0);
        LiveWindow.addActuator("Shooter", "Shooter", controller);
    }
    public double getRPM(){
    	return -shooter2.getEncVelocity() * 3.0 / 1024 * 60;
    }
    public void setRPM(int targetRPM){
        controller.setSetpoint(targetRPM);
        controller.enable();
        controller.setInputRange(-6500, 6500);
        controller.setOutputRange(-1, 1);
        controller.setAbsoluteTolerance(0);
        isShooting = true;
    }
    public void resetPID(){
    	isShooting = false;
    	controller.disable();
    }
    public void shoot(){
        //To be written. How will shooter work? 
    	//flap.set(true);
    	shooter1.set(pidOutput);
    	shooter2.set(pidOutput);
    }
    public void stop(){
    	//flap.set(false);
    	shooter1.set(0);
    	shooter2.set(0);
    }
    public void outputData(){
    	SmartDashboard.putNumber("Shooter Speed", getRPM());
    	SmartDashboard.putNumber("Shooter Error", controller.getError());
    }
    @Override public void setPIDSourceType(PIDSourceType pidSource){
    }
    @Override public PIDSourceType getPIDSourceType(){    
    	return PIDSourceType.kDisplacement;
    }
    @Override public double pidGet(){
        return getRPM();
    }
    @Override public void pidWrite(double output){
    	//System.out.println("Output: " + output);
    	//System.out.println("Error: "+controller.getError());
    	pidOutput = output;
    }
}
