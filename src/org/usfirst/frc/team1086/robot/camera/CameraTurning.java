package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import java.util.ArrayList;
import org.opencv.core.MatOfPoint;

public class CameraTurning implements PIDOutput, CVDataHandler {
    PIDController turnController;
    PIDController driveController;
    TargetType tt = TargetType.BOILER;
    double kPturn = 0;
    double kIturn = 0;
    double kDturn = 0;
    double kPdrive = 0;
    double kIdrive = 0;
    double kDdrive = 0;
    static double kToleranceDegrees = 2.0;
    static double kToleranceDistance = 0.1;
    double pidTurn;
    double pidDrive;
    public CameraTurning(){
        setTargetType(TargetType.BOILER);
    }
    public final void setTargetType(TargetType tt){
        this.tt = tt;
        driveController = new PIDController(kPdrive, kIdrive, kDdrive, new PIDSource(){
            @Override public void setPIDSourceType(PIDSourceType pidSource){}
            @Override public PIDSourceType getPIDSourceType(){
                return PIDSourceType.kDisplacement;
            }
            @Override public double pidGet(){
                return tt.c.distance;
            }
        }, d -> pidDrive = d);
        driveController.setInputRange(0, 20);
        driveController.setOutputRange(-1, 1);
        driveController.setAbsoluteTolerance(kToleranceDistance);
        driveController.setContinuous(false);
        driveController.enable();
        turnController = new PIDController(kPturn, kIturn, kDturn, tt.c, this);
        turnController.setInputRange(-Math.PI, Math.PI);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.enable();
    }
    public TargetType getTargetType(){
        return tt;
    }
    public double turnToAngle(){
        return pidTurn;
    }
    public double getDrivePower(){
        return pidDrive;
    }
    @Override public void pidWrite(double output){
        pidTurn = output;
    }
    public static enum TargetType {
        BOILER(new HighGoalFinder()),
        GEAR(new GearGoalFinder());
        CameraCalculator c;
        TargetType(CameraCalculator c){
            this.c = c;
        }
    }
    @Override public void handle(ArrayList<MatOfPoint> m){
        tt.c.handle(m);
    }
}