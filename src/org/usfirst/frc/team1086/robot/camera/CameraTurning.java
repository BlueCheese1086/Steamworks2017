/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 *
 * @author Aaron
 */
public class CameraTurning implements PIDOutput {
    PIDController turnController;
    CameraCalculator pidInput;
    double kP = 0;
    double kI = 0;
    double kD = 0;
    
    static double kToleranceDegrees = 2.0f;
    double pidOutput;
    
    
    public CameraTurning(){
        pidInput = new HighGoalFinder();
        turnController = new PIDController(kP, kI, kD, pidInput, this);
        turnController.setInputRange(-180, 180);
        turnController.setOutputRange(-1, 1);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
    }
    
    public double turnToAngle(){
        return pidOutput;
    }

    @Override
    public void pidWrite(double output) {
        pidOutput = output;
    }
}
