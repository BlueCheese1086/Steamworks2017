
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import org.usfirst.frc.team1086.robot.RobotMap;

public class Agitator {
    CANTalon agitator; 
    public Agitator(){
        agitator = new CANTalon(RobotMap.AGITATOR);
    }
    public void agitate(){
        agitator.set(1);
    }
    public void stop(){
        agitator.set(0);
    }
}
