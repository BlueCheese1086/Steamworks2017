
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import org.usfirst.frc.team1086.robot.RobotMap;

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
