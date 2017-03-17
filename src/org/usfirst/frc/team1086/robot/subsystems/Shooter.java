
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;

import org.usfirst.frc.team1086.robot.RobotMap;

public class Shooter implements PIDSource, PIDOutput {
    CANTalon shooter1, shooter2;
    PIDController controller = new PIDController(0, 0, 0, this, this);
    Solenoid flap = new Solenoid(RobotMap.FLAP);
    public Shooter(){
        shooter1 = new CANTalon(RobotMap.SHOOTER1);
        shooter2 = new CANTalon(RobotMap.SHOOTER2);
    }
    public int getRPM(){
        return 0;
    }
    public void setRPM(int targetRPM){
        controller.setSetpoint(targetRPM);
        controller.enable();
        controller.setInputRange(0, 18730);
        controller.setOutputRange(-1, 1);
        controller.setAbsoluteTolerance(5);
    }
    public void shoot(){
        //To be written. How will shooter work? 
    	flap.set(true);
    	shooter1.set(.75);
    	shooter2.set(.75);
    }
    public void stop(){
    	flap.set(false);
    	shooter1.set(0);
    	shooter2.set(0);
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
    }
}
