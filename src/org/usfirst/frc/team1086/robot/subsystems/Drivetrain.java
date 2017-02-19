
package org.usfirst.frc.team1086.robot.subsystems;

import com.ctre.CANTalon;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import org.usfirst.frc.team1086.robot.RobotMap;

public class Drivetrain {
    CANTalon leftFrontMecanum, rightFrontMecanum, leftRearMecanum, rightRearMecanum;
    CANTalon leftFrontColson, rightFrontColson, leftRearColson, rightRearColson;
    Solenoid trigger;
    Gyro navX;
    //PIDController controller = new PIDController(0, 0, 0, this, this);
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
        leftFrontMecanum.setInverted(true);
        leftFrontColson.setInverted(true);
        leftRearMecanum.setInverted(true);
        leftRearColson.setInverted(true);
        navX = new Gyro();
        turnToAngleController = new PIDController(0, 0, 0, navX, v -> turnToAngleOutput = v);
        driveStraightController = new PIDController(0, 0, 0, navX, v -> turnToAngleOutput = v);
    }
    public void drive(double leftY, double leftX, double rightX, boolean trigger){
        this.trigger.set(trigger);
        //targetAngle = getGyroAngle();
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
            navX.reset();
            driveStraightController.setSetpoint(0);
            driveStraightController.setAbsoluteTolerance(0.5);
            driveStraightController.setContinuous(true);
            driveStraightController.setInputRange(-180, 180);
            driveStraightController.setOutputRange(-1, 1);
            driveStraightController.enable();
            driveStraight = true;
        }
    }
    public double getTurnPower(){
        if(turnToAngle)
            return turnToAngleOutput;
        else
            return driveStraightOutput;
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
        SmartDashboard.putNumber("leftFrontCim", (leftY - rightX - leftX));
        SmartDashboard.putNumber("leftFrontMini", (leftY - rightX - leftX));
        SmartDashboard.putNumber("rightFrontCim", (leftY + rightX + leftX));
        SmartDashboard.putNumber("rightFrontMini", (leftY + rightX + leftX));
        SmartDashboard.putNumber("leftRearCim", (leftY - rightX + leftX));
        SmartDashboard.putNumber("leftRearMini", (leftY - rightX + leftX));
        SmartDashboard.putNumber("rightRearCim", (leftY + rightX - leftX));
        SmartDashboard.putNumber("rightRearMini", (leftY + rightX - leftX));
        SmartDashboard.putNumber("_leftFrontCimCurrent", leftFrontMecanum.getOutputVoltage());
        SmartDashboard.putNumber("_leftFrontMiniCurrent", leftFrontColson.getOutputVoltage());
        SmartDashboard.putNumber("_rightFrontCimCurrent", rightFrontMecanum.getOutputVoltage());
        SmartDashboard.putNumber("_rightFrontMiniCurrent", rightFrontColson.getOutputVoltage());
        SmartDashboard.putNumber("_rightRearCimCurrent", rightRearMecanum.getOutputVoltage());
        SmartDashboard.putNumber("_rightRearMiniCurrent", rightRearColson.getOutputVoltage());
        SmartDashboard.putNumber("_leftRearCimCurrent", leftRearMecanum.getOutputVoltage());
        SmartDashboard.putNumber("_leftRearMiniCurrent", leftRearColson.getOutputVoltage());
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
        return normalizeAngle(navX.getAngle());
    }
    public double normalizeAngle(double d){
        double ang = (d % 360 + 360 % 360);
        return ang > 180 ? ang - 360 : ang;
    }
}