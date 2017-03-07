
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import org.usfirst.frc.team1086.robot.RobotMap;

public class Agitator {
    CANTalon agitator; 
    public Agitator(){
        agitator = new CANTalon(RobotMap.AGITATOR);
    }
    public void agitate(){
    	double dir = (((long)System.currentTimeMillis() % 6000) / 3000) * 2 - 1;
        agitator.set(0.75 * dir);
    }
    public void stop(){
        agitator.set(0);
    }
}
