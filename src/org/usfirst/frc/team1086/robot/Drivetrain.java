package org.usfirst.frc.team1086.robot;


import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Drivetrain {
    CANTalon leftFrontMecanum, rightFrontMecanum, leftRearMecanum, rightRearMecanum;
    CANTalon leftFrontColson, rightFrontColson, leftRearColson, rightRearColson;
    Solenoid leftTrigger, rightTrigger;
    Gyro gyro;
    double targetAngle = 0;
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
        gyro = new AnalogGyro(RobotMap.GYRO);
    }
    public void drive(double leftY, double leftX, double rightX, boolean trigger){
        leftTrigger.set(trigger);
        rightTrigger.set(trigger);
        targetAngle = getGyroAngle();
        if(!trigger){
            mecanum(leftY, leftX, rightX);
        } else {
            colson(leftY, rightX);
        }
    }
    public void mecanum(double leftY, double leftX, double rightX){
        leftFrontMecanum.set(leftY + rightX - leftX);
        rightFrontMecanum.set(leftY - rightX + leftX);
        leftRearMecanum.set(leftY + rightX + leftX);
        rightRearMecanum.set(leftY - rightX - leftX);
    }
    public void colson(double leftY, double rightX){
            leftFrontMecanum.set(leftY + rightX);
            rightFrontMecanum.set(leftY - rightX);
            leftRearMecanum.set(leftY + rightX);
            rightRearMecanum.set(leftY - rightX);
    }
    public void gyroDrive(double leftY, double leftX, boolean trigger){
        leftTrigger.set(trigger);
        rightTrigger.set(trigger);
        double deltaAngle = normalizeAngle(targetAngle - gyro.getAngle());
        double turnPower = deltaAngle / 90;
        if(!trigger){
            mecanum(leftY, leftX, turnPower);
        } else {
            colson(leftX, turnPower);
        }
    }
    public double getGyroAngle(){
        return normalizeAngle(gyro.getAngle());
    }
    public double normalizeAngle(double d){
        double ang = (d % 360 + 360 % 360);
        return ang > 180 ? ang - 360 : ang;
    }
}
