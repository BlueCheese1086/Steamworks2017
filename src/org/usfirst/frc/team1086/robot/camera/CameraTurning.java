package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import org.opencv.core.MatOfPoint;

public class CameraTurning implements PIDOutput, CVDataHandler {
    PIDController turnController;
    PIDController driveController;
    TargetType tt = TargetType.BOILER;
    double kPturn = -0.015;
    double kIturn = -0.00001;
    double kDturn = -0.017;
    double kPdrive = 0.11;
    double kIdrive = 0.0;
    double kDdrive = 0.075;
    static double kToleranceDegrees = 0.5;
    static double kToleranceDistance = 0.1;
    double pidTurn;
    double pidDrive;
    public double getDistance(){
    	return tt.c.distance;
    }
    public double getAngle(){
    	return tt.c.angle;
    }
    public CameraTurning(){
        setTargetType(TargetType.BOILER);
    }
    public void reset(){
    	driveController.reset();
    	turnController.reset();
    }
    public final void setTargetType(TargetType tt){
        this.tt = tt;
        driveController = new PIDController(kPdrive, kIdrive, kDdrive, new PIDSource(){
            @Override public void setPIDSourceType(PIDSourceType pidSource){}
            @Override public PIDSourceType getPIDSourceType(){
                return PIDSourceType.kDisplacement;
            }
            @Override public double pidGet(){
                return tt.c.distance / 12;
            }
        }, d -> pidDrive = d);
        driveController.setInputRange(0, 20);
        driveController.setOutputRange(-0.25, 0.25);
        driveController.setAbsoluteTolerance(kToleranceDistance);
        driveController.setSetpoint(0.25);
        driveController.setContinuous(false);
        driveController.enable();
        LiveWindow.addActuator("DriveSystem", "Gear Drive", driveController);
        turnController = new PIDController(kPturn, kIturn, kDturn, tt.c, this);
        turnController.setInputRange(-33, 33);
        turnController.setOutputRange(-0.5, 0.5);
        turnController.setSetpoint(0);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(false);
        turnController.enable();
    }
    public TargetType getTargetType(){
        return tt;
    }
    public double turnToAngle(){
    	turnController.enable();
    	if(turnController.getAvgError() > 30){
    		turnController.reset();
    		if(turnController.getError() > 40)
    			return 0.3;
    		else return 0.3 * (turnController.getError() / 33);
    	}
        return turnController.get();
    }
    public double getDrivePower(){
    	driveController.enable();
        return driveController.get();
    }
    @Override public void pidWrite(double output){
        pidTurn = output;
    }
    public static enum TargetType implements PIDSource {
        BOILER(new HighGoalFinder()),
        GEAR(new GearGoalFinder());
        CameraCalculator c;
        TargetType(CameraCalculator c){
            this.c = c;
        }
        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {

        }
        @Override
        public PIDSourceType getPIDSourceType() {
            return PIDSourceType.kDisplacement;
        }
        @Override
        public double pidGet() {
            return c.angle;
        }
    }
    @Override public void handle(ArrayList<MatOfPoint> m){
        tt.c.handle(m);
    }
    public void outputData(){
        SmartDashboard.putString("Current Target Type", tt == TargetType.BOILER ? "Boiler" : "Gear");
        SmartDashboard.putNumber("Distance to Target", tt.c.distance);
        SmartDashboard.putNumber("Angle to Target", tt.c.angle);
        SmartDashboard.putNumber("PID Target Turn Rate", pidTurn);
        SmartDashboard.putNumber("PID Target Drive Rate", pidDrive);
    }
}