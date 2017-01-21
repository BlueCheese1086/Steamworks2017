
package org.usfirst.frc.team1086.robot;


import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Drivetrain implements PIDSource, PIDOutput {
    CANTalon leftFrontMecanum, rightFrontMecanum, leftRearMecanum, rightRearMecanum;
    CANTalon leftFrontColson, rightFrontColson, leftRearColson, rightRearColson;
    Solenoid leftTrigger, rightTrigger;
    Gyro gyro;
    PIDController controller = new PIDController(0, 0, 0, this, this);
    double targetAngle = 0;
    boolean gyroEnabled = false;
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
        if(!gyroEnabled){
            setAngle(gyro.getAngle());
            gyroEnabled = true;
        }
        if(!trigger){
            mecanum(leftY, leftX, getTurnPower());
        } else {
            colson(leftX, getTurnPower());
        }
    }
    public void setAngle(double d){
        targetAngle = normalizeAngle(d);
        controller.setSetpoint(targetAngle);
        controller.enable();
        controller.setContinuous();
        controller.setInputRange(-180, 180);
        controller.setOutputRange(-1, 1);
        controller.setTolerance(5);
    }
    public double getGyroAngle(){
        return normalizeAngle(gyro.getAngle());
    }
    public double getTurnPower(){
        return controller.get();
    }
    public double normalizeAngle(double d){
        double ang = (d % 360 + 360 % 360);
        return ang > 180 ? ang - 360 : ang;
    }
    
    @Override public void setPIDSourceType(PIDSourceType pidSource){}
    @Override public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }
    @Override public double pidGet(){
        return getGyroAngle();
    }
    @Override public void pidWrite(double output){}
}