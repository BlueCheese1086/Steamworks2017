
package org.usfirst.frc.team1086.robot;

import com.ctre.CANTalon;

public class Climber {
    CANTalon climb1;
    public Climber(){
        climb1 = new CANTalon(RobotMap.CLIMB1);
    }
    public void climb(){
        climb1.set(1);
    }
}
