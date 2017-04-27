package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import org.opencv.core.MatOfPoint;

public class CameraTurning implements CVDataHandler {
    PIDController turnController;
    PIDController driveController;
    PIDController strafeController;
    public CameraCalculator calculator;
    static double kPturn = -0.015;
    static double kIturn = -0.00001;
    static double kDturn = -0.017;
    static double kPdrive = -0.21;
    static double kIdrive = -0.0;
    static double kDdrive = -0.28;
    static double kToleranceDegrees = 0.5;
    static double kToleranceDistance = 0.1;
    double pidTurn;
    double pidDrive;
    public double getDistance(){
    	return calculator.distance;
    }
    public double getAngle(){
    	return calculator.angle;
    }
    public CameraTurning(){
    	calculator = new GearGoalFinder();
    	this.driveController = new PIDController(kPdrive, kIdrive, kDdrive, new PIDSource(){
	        @Override public void setPIDSourceType(PIDSourceType pidSource){}
	        @Override public PIDSourceType getPIDSourceType(){
	            return PIDSourceType.kDisplacement;
	        }
	        @Override public double pidGet(){
	            return calculator.distance / 12;
	        }
	    }, d -> {});
	    driveController.setInputRange(0, 20);
	    driveController.setOutputRange(-0.3, 0.3);
	    driveController.setAbsoluteTolerance(kToleranceDistance);
	    driveController.setSetpoint(0.25);
	    driveController.setContinuous(false);
	    driveController.enable();
    }
    public void reset(){
    	driveController.reset();
    }
    public double getDrivePower(){
    	driveController.enable();
        return driveController.get();
    }
    @Override public void handle(ArrayList<MatOfPoint> m){
        calculator.handle(m);
    }
    public void outputData(){
        SmartDashboard.putNumber("Distance to Target", calculator.distance);
        SmartDashboard.putNumber("Angle to Target", calculator.angle);
        SmartDashboard.putNumber("PID Target Turn Rate", pidTurn);
        SmartDashboard.putNumber("PID Target Drive Rate", pidDrive);
    }
}