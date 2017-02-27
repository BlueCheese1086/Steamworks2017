package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.HashMap;
import org.usfirst.frc.team1086.robot.autonomous.*;
import org.usfirst.frc.team1086.robot.subsystems.*;
import org.usfirst.frc.team1086.robot.camera.*;
import org.usfirst.frc.team1086.robot.camera.CameraTurning.TargetType;

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
    AutonomousRoutine chaseLogan;
    CameraTurning targetFinder;
    Shooter flyWheel;
    ImageProcessing imageProcessing;
    Intake intake;
    Climber climber;
    Agitator agitator;
    Compressor compressor;
    Gyro navX;
    PIDController gearDriver;
    double gearDriveOutput;
    SendableChooser <AutonomousRoutine> chooser = new SendableChooser<>();
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
        chooser.addDefault("Logan Chaser", easyGear);
        chooser.addObject("RightGear", rightGear);
        chooser.addObject("Left Gear", leftGear);
        SmartDashboard.putData("Autonomous Chooser", chooser);
        imageProcessing = new ImageProcessing();
        imageProcessing.setCameraTarget(targetFinder);
        imageProcessing.start();
        targetFinder.setTargetType(TargetType.GEAR);
        navX = new Gyro();
        compressor = new Compressor(RobotMap.COMPRESSOR);
        compressor.setClosedLoopControl(true);
        navX.reset();
        gearDriver = new PIDController(-0.028, 0, -.056, 0, targetFinder.getTargetType(), v -> gearDriveOutput = v);
		gearDriver.setInputRange(-180.0, 180.0);
		gearDriver.setOutputRange(-1.0, 1.0);
		gearDriver.setAbsoluteTolerance(.1);
		gearDriver.setContinuous(true);
		gearDriver.enable();
		LiveWindow.addActuator("Gear Driver", "PID", gearDriver);
        defineAutonomousActions();
    }
    public void defineAutonomousActions(){
        actions.put("Drive Forward", () -> drive.drive(0.5, 0, 0, false));
        actions.put("Drive Backwards", () -> drive.drive(-0.5, 0, 0, false));
        actions.put("Stop Fast", () -> drive.drive(-0.2, 0, 0, false));
        actions.put("Stop", () -> drive.drive(0, 0, 0, false));
        actions.put("Test", () -> System.out.println("This is a section"));
        startActions.put("Set Target Turn To 60 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() + 60);
        });
        startActions.put("Set Target Turn To 300 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() - 60);
        });
        startActions.put("Enable Gear Drive", () -> {
        	System.out.println("ENABLED GEAR DRIVE!");
        	gearDriver.setSetpoint(0);
        	gearDriver.enable();
        });
        endActions.put("Reset PID", () -> {
            drive.resetPIDs();
            return true;
        });
        endActions.put("Turn To Target Angle", () -> {
            drive.drive(0, 0, drive.getTurnPower(), false);
            return drive.getActiveController().onTarget();
        });
        endActions.put("Drive to Sight", () -> {
        	targetFinder.setTargetType(TargetType.GEAR);
			drive.mecanum(0.2, 0, 0);
			//System.out.println("SIGHT, Distance: " + targetFinder.getDistance() + " and " + (targetFinder.getDistance() != -1 && targetFinder.getDistance() < 100));
			return targetFinder.getDistance() != -1 && targetFinder.getDistance() < 250;
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
        endActions.put("Chase Logan", () -> {
        	gearDriver.enable();
            if(targetFinder.getTargetType() != CameraTurning.TargetType.GEAR)
                targetFinder.setTargetType(CameraTurning.TargetType.GEAR);
            double drivePower = targetFinder.getDrivePower();           
            drive.drive(Math.signum(drivePower) * Math.sqrt(Math.abs(drivePower)), 0.0, gearDriveOutput, false);
            SmartDashboard.putNumber("Gear Drive Angle", gearDriver.getError());
            SmartDashboard.putNumber("Gear Drive Turn", gearDriveOutput);
            return targetFinder.getDistance() < 34 && targetFinder.getDistance() != -1;
        });
        chaseLogan = new AutonomousRoutine(){
        	@Override public void init(){
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
        	}
        };
        easyGear = new AutonomousRoutine(){
            @Override public void init(){
            	addSection(endActions.get("Drive to Sight"));
                addSection(chaseLogan);
            }
        };
        leftGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(2000, actions.get("Drive Forward"));
                addSection(endActions.get("reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 60 Degrees"));
                addSection(endActions.get("reset PID"));
                addSection(chaseLogan);
            }
        };
        rightGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(2000, actions.get("Drive Forward"));
                addSection(endActions.get("reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 300 Degrees"));
                addSection(endActions.get("reset PID"));
                addSection(chaseLogan);
            }
        };
    }
    @Override public void autonomousInit(){/*
    	chooser.getSelected();
    	chooser.addDefault("LOGAN CHASE", easyGear);
    	System.out.println("TEST CODE: " + SmartDashboard.getString("TEStING", "uh oh"));
        chooser.getSelected().begin();*/
        easyGear.begin();
    }
    @Override public void autonomousPeriodic(){}
    @Override public void teleopInit(){
    	drive.setTurnToAngle(90);
    }
    @Override public void testPeriodic(){
    	teleopPeriodic();
    }
    @Override public void teleopPeriodic(){
        outputData();
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
        	System.out.println("Hello");
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
        
        if(rightStick.getRawButton(5)){
        	drive.getGyro().reset();
        	drive.resetPIDs();
        }
        //Temp code
        if(rightStick.getRawButton(4)){
        	if(!drive.turnToAngle){
        		drive.getGyro().reset();
        		drive.setTurnToAngle(-90);
        	}
        	else {
        		drive.drive(0, 0, drive.getTurnPower(), false);
        	}		
        }
        if(rightStick.getRawButton(3)){
        	if(!drive.driveStraight){
        		drive.getGyro().reset();
        		drive.startDriveStraight();
        	}
        	else {
        		drive.mecanum(leftStick.getY() * mult, leftStick.getX() * mult, drive.getTurnPower());
        	}
        }
        if(!rightStick.getRawButton(4) && !rightStick.getRawButton(3))
        	drive.resetPIDs();
        
        if(rightStick.getRawButton(7)){
			if(targetFinder.getTargetType() != CameraTurning.TargetType.GEAR)
				targetFinder.setTargetType(CameraTurning.TargetType.GEAR);
			if(!gearDriver.isEnabled())
				gearDriver.enable();
			System.out.println("Distance: " + targetFinder.getDistance() + " " + (targetFinder.getDistance() < 40 && targetFinder.getDistance() != -1));
            System.out.println("DRIVE POWER: " + targetFinder.getDrivePower());
            System.out.println("GEAR TURN POWER: " + gearDriveOutput);
			drive.mecanum(targetFinder.getDrivePower(), 0, gearDriveOutput * 0.5);
		}
    }
    private void outputData(){
        navX.outputData();
        drive.outputPIDData();
        targetFinder.outputData();
        SmartDashboard.putNumber("Gear Drive Output", gearDriveOutput);
        SmartDashboard.putNumber("Gear Drive Speed", targetFinder.getDrivePower());
    }
}
