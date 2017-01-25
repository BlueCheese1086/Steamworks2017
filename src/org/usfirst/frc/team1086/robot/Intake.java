
package org.usfirst.frc.team1086.robot;

import com.ctre.CANTalon;

public class Intake {
    CANTalon intake;
    public Intake(){
        intake = new CANTalon(RobotMap.INTAKE);
    }
    public void motorIn(){
        intake.set(1);
    }
    public void motorOff(){
        intake.set(0);
    }
}
