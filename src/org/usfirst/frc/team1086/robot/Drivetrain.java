package org.usfirst.frc.team1086.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Solenoid;

public class Drivetrain {
    CANTalon leftFrontMecanum, rightFrontMecanum, leftRearMecanum, rightRearMecanum;
    CANTalon leftFrontColson, rightFrontColson, leftRearColson, rightRearColson;
    Solenoid leftTrigger, rightTrigger;
    public Drivetrain(){
        leftFrontMecanum = new CANTalon(RobotMap.LEFT_FRONT_MECANUM);
        leftRearMecanum = new CANTalon(RobotMap.LEFT_REAR_MECANUM);
        rightFrontMecanum = new CANTalon(RobotMap.RIGHT_FRONT_MECANUM);
        rightRearMecanum = new CANTalon(RobotMap.RIGHT_REAR_MECANUM);
        leftFrontColson = new CANTalon(RobotMap.LEFT_FRONT_COLSON);
        leftRearColson = new CANTalon(RobotMap.LEFT_REAR_COLSON);
        rightFrontColson = new CANTalon(RobotMap.RIGHT_FRONT_COLSON);
        rightRearColson = new CANTalon(RobotMap.RIGHT_REAR_COLSON);
        
        leftTrigger = new Solenoid(RobotMap.LEFT_TRIGGER);
        rightTrigger = new Solenoid(RobotMap.RIGHT_TRIGGER);
    }
    public void drive(double leftY, double leftX, double rightX, boolean trigger){
        leftTrigger.set(trigger);
        rightTrigger.set(trigger);
        if(!trigger){
            leftFrontMecanum.set(leftY + rightX - leftX);
            rightFrontMecanum.set(leftY - rightX + leftX);
            leftRearMecanum.set(leftY + rightX + leftX);
            rightRearMecanum.set(leftY - rightX - leftX);
        } else {
            leftFrontMecanum.set(leftY + rightX);
            rightFrontMecanum.set(leftY - rightX);
            leftRearMecanum.set(leftY + rightX);
            rightRearMecanum.set(leftY - rightX);
        }
    }
    
}
