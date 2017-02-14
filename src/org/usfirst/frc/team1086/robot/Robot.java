package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import java.util.HashMap;
import org.usfirst.frc.team1086.robot.autonomous.*;
import org.usfirst.frc.team1086.robot.subsystems.*;
import org.usfirst.frc.team1086.robot.camera.*;

public class Robot extends IterativeRobot {
    Drivetrain drive;
    Joystick leftStick;
    Joystick rightStick;
    Joystick auxiliaryStick;
    HashMap<String, Runnable> actions = new HashMap();
    HashMap<String, Runnable> startActions = new HashMap();
    HashMap<String, Action> endActions = new HashMap();
    AutonomousRoutine easyGear;
    AutonomousRoutine leftGear;
    AutonomousRoutine rightGear;
    CameraTurning targetFinder;
    Shooter flyWheel;
    ImageProcessing imageProcessing;
    Intake intake;
    Climber climber;
    Agitator agitator;
    boolean buttonDown = false;
    boolean backward = false;
    @Override public void robotInit(){
        drive = new Drivetrain();
        leftStick = new Joystick(RobotMap.LEFT_STICK);
        rightStick = new Joystick(RobotMap.RIGHT_STICK);
        auxiliaryStick = new Joystick(RobotMap.AUXILIARY_STICK);
        targetFinder = new CameraTurning();
        flyWheel = new Shooter();
        intake = new Intake();
        climber = new Climber();
        agitator = new Agitator();
        //imageProcessing = new ImageProcessing();
        //imageProcessing.setCameraTarget(targetFinder);
        //imageProcessing.start();
        //defineAutonomousActions();
    }
    public void defineAutonomousActions(){
        actions.put("Drive Forward", () -> drive.drive( 1, 0, 0, false));
        actions.put("Drive Backwards", () -> drive.drive(-1, 0, 0, false));
        startActions.put("Set Target Turn To 60 Degrees", () -> {
            drive.setAngle(drive.getGyroAngle() + 60);
        });
        startActions.put("Set Target Turn To 300 Degrees", () -> {
            drive.setAngle(drive.getGyroAngle() - 60);
        });
        endActions.put("Turn To Target Angle", () -> {
            drive.drive(0, 0, drive.getController().get(), false);
            return drive.getController().onTarget();
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
        easyGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(endActions.get("Drive To Gear"));
            }
        };
        leftGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(2000, actions.get("Drive Forward"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 60 Degrees"));
                addSection(endActions.get("Drive To Gear"));
            }
        };
        rightGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(2000, actions.get("Drive Forward"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 300 Degrees"));
                addSection(endActions.get("Drive To Gear"));
            }
        };
    }
    @Override public void autonomousInit(){
        easyGear.begin();
    }
    @Override public void autonomousPeriodic(){}
    @Override public void teleopInit(){}
    @Override public void teleopPeriodic(){
        if(!buttonDown && leftStick.getRawButton(2))
            backward = !backward;
        buttonDown = leftStick.getRawButton(2);
        int mult = backward ? -1 : 1;
        if(leftStick.getRawButton(ButtonMap.SAFETY_DRIVE)){
            if(!rightStick.getRawButton(ButtonMap.GYRO_DRIVE))
                drive.drive(leftStick.getY() * mult, leftStick.getX() * mult, rightStick.getX(), rightStick.getRawButton(ButtonMap.OCTO_SHIFTER));
            else
                drive.gyroDrive(leftStick.getY() * mult, leftStick.getX(), rightStick.getRawButton(ButtonMap.OCTO_SHIFTER));
        }
        else drive.drive(0, 0, 0, rightStick.getRawButton(ButtonMap.OCTO_SHIFTER));
        if(auxiliaryStick.getRawButton(ButtonMap.SHOOT)){
            flyWheel.shoot();
        }
        if(auxiliaryStick.getRawButton(ButtonMap.COLLECT)){
            intake.motorIn();
        } 
        else {
            intake.motorOff();
        }
        
        if(auxiliaryStick.getRawButton(ButtonMap.CLIMB)){
            climber.climb();
        }
        else {
            climber.stop();
        }
        if(auxiliaryStick.getRawButton(ButtonMap.AGITATE)){
            agitator.agitate();
        }
        else { 
            agitator.stop();
        }
    }
    @Override public void testPeriodic(){}
}
