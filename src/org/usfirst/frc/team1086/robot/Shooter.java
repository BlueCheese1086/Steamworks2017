
package org.usfirst.frc.team1086.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class Shooter implements PIDSource, PIDOutput {
    CANTalon shooter;
    PIDController controller = new PIDController(0, 0, 0, this, this);
    public Shooter(){
        shooter = new CANTalon(RobotMap.SHOOTER);
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
        shooter.set(output);
    }
}
