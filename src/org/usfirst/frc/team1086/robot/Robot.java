package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import java.util.HashMap;
import org.usfirst.frc.team1086.robot.autonomous.*;
import org.usfirst.frc.team1086.robot.camera.*;

public class Robot extends IterativeRobot {
    Drivetrain drive;
    Joystick leftStick;
    Joystick rightStick;
    Joystick auxiliaryStick;
    HashMap<String, Runnable> actions = new HashMap();
    HashMap<String, Runnable> startActions = new HashMap();
    HashMap<String, Action> endActions = new HashMap();
    AutonomousRoutine getToGear;
    CameraTurning targetFinder;
    Shooter flyWheel;
    ImageProcessing imageProcessing;
    Intake intake;
    Climber motor1;
    Climber motor2;
    @Override public void robotInit(){
        drive = new Drivetrain();
        leftStick = new Joystick(RobotMap.LEFT_STICK);
        rightStick = new Joystick(RobotMap.RIGHT_STICK);
        auxiliaryStick = new Joystick(RobotMap.AUXILIARY_STICK);
        targetFinder = new CameraTurning();
        flyWheel = new Shooter();
        imageProcessing = new ImageProcessing();
        imageProcessing.setCameraTarget(targetFinder);
        imageProcessing.start();
        defineAutonomousActions();
    }
    public void defineAutonomousActions(){
        actions.put("Drive Forward", () -> drive.drive( 1, 0, 0, false));
        actions.put("Drive Backwards", () -> drive.drive(-1, 0, 0, false));
        startActions.put("Set Target Turn To 90 Degrees", () -> {
            drive.setAngle(drive.getGyroAngle() + 90);
        });
        startActions.put("Set Target Turn To 270 Degrees", () -> {
            drive.setAngle(drive.getGyroAngle() - 90);
        });
        endActions.put("Turn To Target Angle", () -> {
            drive.drive(0, 0, drive.controller.get(), false);
            return drive.controller.onTarget();
        });
        endActions.put("Turn To Boiler", () -> {
            if(targetFinder.getTargetType() != CameraTurning.TargetType.BOILER)
                targetFinder.setTargetType(CameraTurning.TargetType.BOILER);
            drive.drive(0, 0, targetFinder.turnToAngle(), false);
            return targetFinder.turnToAngle() == 0;
        });
        endActions.put("Turn To Gear", () -> {
            if(targetFinder.getTargetType() != CameraTurning.TargetType.GEAR)
                targetFinder.setTargetType(CameraTurning.TargetType.GEAR);
            drive.drive(0, 0, targetFinder.turnToAngle(), false);
            return targetFinder.turnToAngle() == 0;
        });
        endActions.put("Drive To Gear", () -> {
            if(targetFinder.getTargetType() != CameraTurning.TargetType.GEAR)
                targetFinder.setTargetType(CameraTurning.TargetType.GEAR);
            drive.drive(targetFinder.getDrivePower(), 0, targetFinder.turnToAngle(), false);
            return targetFinder.turnToAngle() == 0;
        });
        getToGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(1000, actions.get("Drive Forward"));
                addSection(endActions.get("Turn To Angle"), startActions.get("Set Target Turn To 90 Degrees"));
                addSection(1000, actions.get("Drive Forward"));
                addSection(endActions.get("Turn To Angle"), startActions.get("Set Target Turn To 270 Degrees"));
                addSection(endActions.get("Drive To Gear"));
            }
        };
    }
    @Override public void autonomousInit(){
        getToGear.begin();
    }
    @Override public void autonomousPeriodic(){}
    @Override public void teleopInit(){
    }
    @Override public void teleopPeriodic(){
        if(leftStick.getRawButton(1)){
            if(!rightStick.getRawButton(2))
                drive.drive(leftStick.getY(), leftStick.getX(), rightStick.getX(), rightStick.getRawButton(1));
            else
                drive.gyroDrive(leftStick.getY(), leftStick.getX(), rightStick.getRawButton(1));
        }
        else drive.drive(0, 0, 0, rightStick.getRawButton(1));
        if(auxiliaryStick.getRawButton(ButtonMap.SHOOT)){
            flyWheel.shoot();
        }
        if(auxiliaryStick.getRawButton(ButtonMap.COLLECT)){
            intake.motorIn();
        }
        if(auxiliaryStick.getRawButton(ButtonMap.COLLECT) == false) {
            intake.motorOff();
        }
        if(auxiliaryStick.getRawButton(ButtonMap.CLIMB)){
             motor1.climb();
             motor2.climb();
        }
    }
    @Override public void testPeriodic(){}
}