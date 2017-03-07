
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Compressor;
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
    public PIDController driveStraightController;
    double turnToAngleOutput;
    double driveStraightOutput;
    public boolean turnToAngle = false;
    public boolean driveStraight = false;
    public Drivetrain(){
        leftFrontMecanum = new CANTalon(RobotMap.LEFT_FRONT_MECANUM);
        leftRearMecanum = new CANTalon(RobotMap.LEFT_REAR_MECANUM);
        rightFrontMecanum = new CANTalon(RobotMap.RIGHT_FRONT_MECANUM);
        rightRearMecanum = new CANTalon(RobotMap.RIGHT_REAR_MECANUM);      
        leftFrontColson = new CANTalon(RobotMap.LEFT_FRONT_COLSON);
        leftRearColson = new CANTalon(RobotMap.LEFT_REAR_COLSON);
        rightFrontColson = new CANTalon(RobotMap.RIGHT_FRONT_COLSON);
        rightRearColson = new CANTalon(RobotMap.RIGHT_REAR_COLSON);
        leftFrontMecanum.setInverted(true);
        leftFrontColson.setInverted(true);
        leftRearMecanum.setInverted(true);
        leftRearColson.setInverted(true);
        trigger = new Solenoid(RobotMap.TRIGGER);
        
        navX = new Gyro();
        turnToAngleController = new PIDController(0.058, 0, 0.048, navX, v -> turnToAngleOutput = v);
        driveStraightController = new PIDController(0.063, 0, 0, navX, v -> driveStraightOutput = v);
    }
    public void drive(double leftY, double leftX, double rightX, boolean trigger){
        this.trigger.set(trigger);
        if(!trigger){
            mecanum(leftY * Math.abs(leftY), leftX * Math.abs(leftX), rightX * Math.abs(rightX));
        } else {
            colson(leftY * Math.abs(leftY), rightX * Math.abs(rightX));
        }
    }
    public Gyro getGyro(){
        return navX;
    }
    public void setTurnToAngle(double angle){
        if(!turnToAngle){
            turnToAngleController.setSetpoint(angle);
            turnToAngleController.setAbsoluteTolerance(1.5);
            turnToAngleController.setContinuous(true);
            turnToAngleController.setInputRange(-180, 180);
            turnToAngleController.setOutputRange(-0.6, 0.6);
            turnToAngleController.enable();
            turnToAngle = true;
        }
    }
    public void startDriveStraight(){
        if(!driveStraight){
            driveStraightController.setSetpoint(0);
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
        else if(driveStraight){
            return driveStraightOutput;
        }
        else
            return 0;
    }
    
    public void mecanum(double leftY, double leftX, double rightX){
        leftFrontMecanum.set(0.9 * (leftY - rightX - leftX));
        leftFrontColson.set(0.9 * (leftY - rightX - leftX));
        rightFrontMecanum.set(0.9 * (leftY + rightX + leftX));
        rightFrontColson.set(0.9 * (leftY + rightX + leftX));
        leftRearMecanum.set(0.9 * (leftY - rightX + leftX));
        leftRearColson.set(0.9 * (leftY - rightX + leftX));
        rightRearMecanum.set(0.9 * (leftY + rightX - leftX));
        rightRearColson.set(0.9 * (leftY + rightX - leftX));
        
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
        SmartDashboard.putNumber("Set Point", turnToAngleController.getSetpoint());
    }
}