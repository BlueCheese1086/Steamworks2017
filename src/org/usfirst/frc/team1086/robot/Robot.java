package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
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
    HashMap<String, AutonomousRoutine> autoRoutines = new HashMap();
    AutonomousRoutine easyGear;
    AutonomousRoutine leftGear;
    AutonomousRoutine rightGear;
    AutonomousRoutine chaseLogan;
    AutonomousRoutine driveSlowlyForward, driveForward;
    AutonomousRoutine selected;
    final String easy = "Easy Gear";
    final String left = "Left Gear";
    final String right = "Right Gear";
    final String forward8 = "Drive forward for 8 seconds";
    final String forward15 = "Drive forward for 15 seconds";
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
    SendableChooser<String> chooser = new SendableChooser<>();
    Timer t;
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
        t = new Timer();
        climber = new Climber();
        agitator = new Agitator();
        chooser.addDefault("Logan Chaser", easy);
        chooser.addObject("RightGear", right);
        chooser.addObject("Left Gear", left);  
        chooser.addObject("Drive Forward for 8 seconds", forward8);
        chooser.addObject("Drive Forward for 15 seconds", forward15);
        SmartDashboard.putData("Autonomous Chooser", chooser);
        //imageProcessing = new ImageProcessing();
        //imageProcessing.setCameraTarget(targetFinder);
        //imageProcessing.start();
        targetFinder.setTargetType(TargetType.GEAR);
        navX = new Gyro();
        compressor = new Compressor(RobotMap.COMPRESSOR);
        compressor.setClosedLoopControl(true);
        navX.reset();
        gearDriver = new PIDController(-0.0265, 0, -.056, 0, targetFinder.getTargetType(), v -> gearDriveOutput = v);		
        gearDriver.setInputRange(-180.0, 180.0);
		gearDriver.setOutputRange(-1, 1);
		gearDriver.setAbsoluteTolerance(.1);
		gearDriver.setContinuous(true);
		gearDriver.enable();
		LiveWindow.addActuator("Gear Driver", "PID", gearDriver);
        defineAutonomousActions();
        autoRoutines.put(easy, easyGear);
        autoRoutines.put(left, leftGear);
        autoRoutines.put(right, rightGear);
        autoRoutines.put(forward8, driveSlowlyForward);
        autoRoutines.put(forward15, driveForward);
    }
    public void defineAutonomousActions(){
        actions.put("Drive Forward", () -> drive.drive(Math.sqrt(0.5), 0, 0, false));
        actions.put("Drive Forward Slowly", () -> drive.drive(Math.sqrt(0.25), 0, 0, false));
        actions.put("Drive Backwards", () -> drive.drive(-Math.sqrt(0.5), 0, 0, false));
        actions.put("Stop Fast", () -> drive.drive(-0.2, 0, 0, false));
        actions.put("Stop", () -> drive.drive(0, 0, 0, false));
        startActions.put("Set Target Turn To 60 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() + 60);
        });
        startActions.put("Set Target Turn To 300 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() - 60);
        });
        startActions.put("Enable Gear Drive", () -> {
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
			drive.mecanum(0.4, 0, 0);
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
            return targetFinder.getDistance() < 30 && targetFinder.getDistance() != -1;
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
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(3000, actions.get("Stop"));
                /*addSection(1000, actions.get("Drive Backward"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 60 Degrees"));
                addSection(1500, actions.get("Drive Forward"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 300 Degrees"));
            	addSection(1000000, actions.get("Drive Forward"));*/
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
        driveSlowlyForward = new AutonomousRoutine(){
        	@Override public void init(){
        		addSection(8000, actions.get("Drive Forward Slowly"));
        		addSection(1000, actions.get("Stop"));
        	}
        };
        driveForward = new AutonomousRoutine(){
        	@Override public void init(){
        		addSection(15000, actions.get("Drive Forward Slowly"));
        		addSection(1000, actions.get("Stop"));
        	}
        };
    }
    @Override public void autonomousInit(){
    	selected = autoRoutines.get(chooser.getSelected());
    	selected.begin();
    	//t.reset();
    	//t.start();
    }
    @Override public void autonomousPeriodic(){
    	selected.tick();
    	//if(t.get() <= 6.5)
    	//	drive.mecanum(0.3, 0, 0);
    	//else drive.mecanum(0, 0, 0);
    }
    @Override public void teleopInit(){
    	drive.setTurnToAngle(180);
    	autoRoutines.get(chooser.getSelected()).stop();
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
        	drive.drive(leftStick.getY() * mult, leftStick.getX() * mult, rightStick.getX(), rightStick.getRawButton(ButtonMap.OCTO_SHIFTER));
        }
        else drive.drive(0, 0, 0, rightStick.getRawButton(ButtonMap.OCTO_SHIFTER));
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
        if(auxiliaryStick.getRawButton(ButtonMap.SHOOT)){
            agitator.agitate();
            flyWheel.shoot();
        }
        else { 
            agitator.stop();
            flyWheel.stop();
        }
        //Temp code
        if(rightStick.getRawButton(ButtonMap.RAPID_TURN)){
        	if(!drive.turnToAngle){
        		drive.getGyro().reset();
        		drive.setTurnToAngle(-180);
        	}
        	else {
        		drive.drive(0, 0, drive.getTurnPower(), false);
        	}
        }
        if(rightStick.getRawButton(ButtonMap.STRAIGHT_DRIVE)){
        	if(!drive.driveStraight){
        		drive.getGyro().reset();
        		drive.startDriveStraight();
        	}
        	else {
        		drive.drive(leftStick.getY() * mult, leftStick.getX() * mult, drive.getTurnPower(), rightStick.getRawButton(ButtonMap.OCTO_SHIFTER));
        	}
        }
        if(!rightStick.getRawButton(ButtonMap.RAPID_TURN) && !rightStick.getRawButton(ButtonMap.STRAIGHT_DRIVE))
        	drive.resetPIDs();
        
        if(rightStick.getRawButton(ButtonMap.GEAR_DRIVE)){
			if(targetFinder.getTargetType() != CameraTurning.TargetType.GEAR)
				targetFinder.setTargetType(CameraTurning.TargetType.GEAR);
			if(!gearDriver.isEnabled())
				gearDriver.enable();
			drive.mecanum(targetFinder.getDrivePower(), 0, gearDriveOutput);
		}
    }
    private void outputData(){
        navX.outputData();
        drive.outputPIDData();
        targetFinder.outputData();
        SmartDashboard.putNumber("Gear Drive Output", gearDriveOutput);
        SmartDashboard.putNumber("Gear Drive Speed", targetFinder.getDrivePower());
        SmartDashboard.putNumber("NEW ANGLE", targetFinder.tt.c.getTargetAngle());
    }
}
