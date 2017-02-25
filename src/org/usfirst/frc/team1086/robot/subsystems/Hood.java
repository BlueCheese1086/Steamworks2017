package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.usfirst.frc.team1086.robot.RobotMap;

public class Hood implements PIDOutput {
    DigitalInput upSwitch;
    DigitalInput downSwitch;
    CANTalon hoodMotor;
    Encoder encoder;
    PIDController controller;
    public Hood(){
        upSwitch = new DigitalInput(RobotMap.UP_SWITCH);
        downSwitch = new DigitalInput(RobotMap.DOWN_SWITCH);
        hoodMotor = new CANTalon(RobotMap.HOOD);
        encoder = new Encoder(RobotMap.HOOD_ENCODER, RobotMap.HOOD_ENCODER + 1);
        controller = new PIDController(0, 0, 0, encoder, this);
        controller.setAbsoluteTolerance(0.1);
        controller.setOutputRange(-1, 1);
    }
    @Override public void pidWrite(double output){
        move(output);
    }
    public void move(double d){
        if(d > 0)
            hoodMotor.set(upSwitch.get() ? 0 : d);
        else hoodMotor.set(downSwitch.get() ? 0 : d); 
    }
}
