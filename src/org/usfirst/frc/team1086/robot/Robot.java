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
    @Override public void autonomousInit(){
        AutonomousRoutine driveForward = new AutonomousRoutine(){
            @Override public void init(){
                ac.addSection(5000, () -> {
                    drive.mecanum(1, 0, 0);
                });
            }
        };//Drives forward for 5 seconds. Can be defined in robotInit
        driveForward.begin();//Run the auto
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