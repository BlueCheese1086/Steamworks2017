package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
    Drivetrain drive;
    Joystick leftStick;
    Joystick rightStick;
    @Override public void robotInit(){
        drive = new Drivetrain();
        leftStick = new Joystick(RobotMap.LEFT_STICK);
        rightStick = new Joystick(RobotMap.RIGHT_STICK);
    }
    @Override public void autonomousInit(){}
    @Override public void autonomousPeriodic(){}
    @Override public void teleopPeriodic(){
        drive.drive(leftStick.getY(), leftStick.getX(), rightStick.getX(), rightStick.getRawButton(1));
    }
    @Override public void testPeriodic(){}
}

