
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import org.usfirst.frc.team1086.robot.RobotMap;

public class Intake {
    CANTalon intake1;
    CANTalon intake2;
    public Intake(){
        intake1 = new CANTalon(RobotMap.INTAKE1);
        intake2 = new CANTalon(RobotMap.INTAKE2);
    }
    public void motorIn(){
        intake1.set(1);
        intake2.set(1);
    }
    public void motorOff(){
        intake1.set(0);
        intake2.set(0);
    }
}
