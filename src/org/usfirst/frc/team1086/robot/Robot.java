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
    HashMap<String, Runnable> actions = new HashMap();
    HashMap<String, Action> endActions = new HashMap();
    AutonomousRoutine driveForwardAndBack;
    CameraTurning targetFinder;
    @Override public void robotInit(){
        drive = new Drivetrain();
        leftStick = new Joystick(RobotMap.LEFT_STICK);
        rightStick = new Joystick(RobotMap.RIGHT_STICK);
        targetFinder = new CameraTurning();
        defineAutonomousActions();
    }
    public void defineAutonomousActions(){
        actions.put("Drive Forward", () -> drive.drive( 1, 0, 0, false));
        actions.put("Drive Backwards", () -> drive.drive(-1, 0, 0, false));
        endActions.put("Turn To Boiler", () -> {
            if(targetFinder.getTargetType() != CameraTurning.TargetType.BOILER)
                targetFinder.setTargetType(CameraTurning.TargetType.BOILER);
            drive.drive(0, 0, targetFinder.turnToAngle(), false);
            return targetFinder.turnToAngle() == 0;
        });
        driveForwardAndBack = new AutonomousRoutine(){
            @Override public void init(){
                addSection(5000, actions.get("Drive Forward"));
                addSection(5000, actions.get("Drive Backwards"));
                addSection(endActions.get("Turn To Boiler"));
            }
        };
    }
    @Override public void autonomousInit(){
        driveForwardAndBack.begin();//Run the auto
    }
    @Override public void autonomousPeriodic(){}
    @Override public void teleopPeriodic(){
        if(leftStick.getRawButton(1)){
            if(!rightStick.getRawButton(2))
                drive.drive(leftStick.getY(), leftStick.getX(), rightStick.getX(), rightStick.getRawButton(1));
            else
                drive.gyroDrive(leftStick.getY(), leftStick.getX(), rightStick.getRawButton(1));
        }
        else drive.drive(0, 0, 0, rightStick.getRawButton(1));
    }
    @Override public void testPeriodic(){}
}