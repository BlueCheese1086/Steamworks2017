
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team1086.robot.RobotMap;

public class Drivetrain {
    CANTalon leftFrontMecanum, rightFrontMecanum, leftRearMecanum, rightRearMecanum;
    CANTalon leftFrontColson, rightFrontColson, leftRearColson, rightRearColson;
    Solenoid trigger;
    Gyro navX;
    PIDController turnToAngleController;
    PIDController driveStraightController;
    double turnToAngleOutput;
    double driveStraightOutput;
    boolean turnToAngle = false;
    boolean driveStraight = false;
    public Drivetrain(){
        leftFrontMecanum = new CANTalon(RobotMap.LEFT_FRONT_MECANUM);
        leftRearMecanum = new CANTalon(RobotMap.LEFT_REAR_MECANUM);
        rightFrontMecanum = new CANTalon(RobotMap.RIGHT_FRONT_MECANUM);
        rightRearMecanum = new CANTalon(RobotMap.RIGHT_REAR_MECANUM);
        leftFrontColson = new CANTalon(RobotMap.LEFT_FRONT_COLSON);
        leftRearColson = new CANTalon(RobotMap.LEFT_REAR_COLSON);
        rightFrontColson = new CANTalon(RobotMap.RIGHT_FRONT_COLSON);
        rightRearColson = new CANTalon(RobotMap.RIGHT_REAR_COLSON);
        trigger = new Solenoid(RobotMap.TRIGGER);
        navX = new Gyro();
        turnToAngleController = new PIDController(0, 0, 0, navX, v -> turnToAngleOutput = v);
        driveStraightController = new PIDController(0, 0, 0, navX, v -> turnToAngleOutput = v);
    }
    public void drive(double leftY, double leftX, double rightX, boolean trigger){
        this.trigger.set(trigger);
        if(!trigger){
            mecanum(leftY, leftX, rightX);
        } else {
            colson(leftY, rightX);
        }
    }
    public Gyro getGyro(){
        return navX;
    }
    public void setTurnToAngle(double angle){
        if(!turnToAngle){
            turnToAngleController.setSetpoint(angle);
            turnToAngleController.setAbsoluteTolerance(0.5);
            turnToAngleController.setContinuous(true);
            turnToAngleController.setInputRange(-180, 180);
            turnToAngleController.setOutputRange(-1, 1);
            turnToAngleController.enable();
            turnToAngle = true;
        }
    }
    public void startDriveStraight(){
        if(!driveStraight){
            driveStraightController.setSetpoint(navX.getAngle());
            driveStraightController.setAbsoluteTolerance(0.5);
            driveStraightController.setContinuous(true);
            driveStraightController.setInputRange(-180, 180);
            driveStraightController.setOutputRange(-1, 1);
            driveStraightController.enable();
            driveStraight = true;
        }
    }
    public void resetPIDs(){
        turnToAngle = false;
        driveStraight = false;
        turnToAngleController.disable();
        driveStraightController.disable();
    }
    public double getTurnPower(){
        if(turnToAngle)
            return turnToAngleOutput;
        else if(driveStraight)
            return driveStraightOutput;
        else
            return 0;
    }
    
    public void mecanum(double leftY, double leftX, double rightX){
        leftFrontMecanum.set(leftY - rightX - leftX);
        leftFrontColson.set(leftY - rightX - leftX);
        rightFrontMecanum.set(leftY + rightX + leftX);
        rightFrontColson.set(leftY + rightX + leftX);
        leftRearMecanum.set(leftY - rightX + leftX);
        leftRearColson.set(leftY - rightX + leftX);
        rightRearMecanum.set(leftY + rightX - leftX);
        rightRearColson.set(leftY + rightX - leftX);
    }
    public void colson(double leftY, double rightX){
        leftFrontMecanum.set(leftY - rightX);
        leftFrontColson.set(leftY - rightX);
        rightFrontMecanum.set(leftY + rightX);
        rightFrontColson.set(leftY + rightX);
        leftRearMecanum.set(leftY - rightX);
        leftRearColson.set(leftY - rightX);
        rightRearMecanum.set(leftY + rightX);
        rightRearColson.set(leftY + rightX);
    } 
    public void gyroDrive(double leftY, double leftX, boolean trigger){
        /*this.trigger.set(trigger);
        if(!gyroEnabled){
            setAngle(navX.getAngle());
            gyroEnabled = true;
        }
        if(!trigger){
            mecanum(leftY, leftX, getTurnPower());
        } else {
            colson(leftX, getTurnPower());
        }*/
    }
    public PIDController getActiveController(){
        if(turnToAngle)
            return turnToAngleController;
        else if(driveStraight)
            return driveStraightController;
        else
            return null;
    }
    public double getGyroAngle(){
        return navX.getNormalizedAngle();
    }
    public void outputPIDData(){
        SmartDashboard.putNumber("PID Turn Rate", getTurnPower());
    }
}