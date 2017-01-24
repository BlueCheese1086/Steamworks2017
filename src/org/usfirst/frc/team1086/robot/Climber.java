
package org.usfirst.frc.team1086.robot;

import com.ctre.CANTalon;

public class Climber {
    CANTalon climb1;
    CANTalon climb2;
    public Climber(){
        climb1 = new CANTalon(RobotMap.CLIMB1);
        climb2 = new CANTalon(RobotMap.CLIMB2);
    }
    public void climb(){
        climb1.set(1);
        climb2.set(1);
    }
}
