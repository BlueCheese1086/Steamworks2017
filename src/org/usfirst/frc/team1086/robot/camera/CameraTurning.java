package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

public class CameraTurning implements PIDOutput {
    PIDController turnController;
    TargetType tt = TargetType.BOILER;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    static double kToleranceDegrees = 2.0f;
    double pidOutput;
    public CameraTurning(){
        turnController = new PIDController(kP, kI, kD, tt.c, this);
        turnController.setInputRange(-180, 180);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.enable();
    }
    public void setTargetType(TargetType tt){
        this.tt = tt;
        turnController = new PIDController(kP, kI, kD, tt.c, this);
        turnController.setInputRange(-180, 180);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        turnController.enable();
    }
    public TargetType getTargetType(){
        return tt;
    }
    public double turnToAngle(){
        return pidOutput;
    }
    @Override public void pidWrite(double output) {
        pidOutput = output;
    }
    public static enum TargetType {
        BOILER(new HighGoalFinder()),
        GEAR(new GearGoalFinder());
        CameraCalculator c;
        TargetType(CameraCalculator c){
            this.c = c;
        }
    }
}
