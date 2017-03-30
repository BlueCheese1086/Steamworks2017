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
    InputManager im;
    GearHolder evictor;
    HashMap<String, Section> actions = new HashMap();
    HashMap<String, Section> startActions = new HashMap();
    HashMap<String, Action> endActions = new HashMap();
    HashMap<String, AutonomousRoutine> autoRoutines = new HashMap();
    AutonomousRoutine easyGear;
    AutonomousRoutine leftGear;
    AutonomousRoutine rightGear;
    AutonomousRoutine chaseLogan;
    AutonomousRoutine driveSlowlyForward, driveForward, noCamAuto;
    AutonomousRoutine selected;
    final String easy = "Easy Gear";
    final String left = "Left Gear";
    final String right = "Right Gear";
    final String forward8 = "Drive forward for 8 seconds";
    final String forward15 = "Drive forward for 15 seconds";
    final String bonus = "Auto with no camera";
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
    Encoders encoders;
    boolean buttonDown = false;
    boolean backward = false;
    boolean gearDrive = false;
    @Override public void robotInit(){
        drive = new Drivetrain();
        drive.resetEncoders();
        evictor = new GearHolder();
        im = new InputManager();
        targetFinder = new CameraTurning();
        flyWheel = new Shooter();
        intake = new Intake();
        t = new Timer();
        climber = new Climber();
        agitator = new Agitator();
        imageProcessing = new ImageProcessing();
        imageProcessing.setCameraTarget(targetFinder);
        imageProcessing.start();
        targetFinder.setTargetType(TargetType.GEAR);
        navX = new Gyro();
        compressor = new Compressor(RobotMap.COMPRESSOR);
        compressor.setClosedLoopControl(true);
        navX.reset();
        gearDriver = new PIDController(-0.025, 0, -.0583, 0, targetFinder.getTargetType(), v -> gearDriveOutput = v);		
        gearDriver.setInputRange(-180.0, 180.0);
		gearDriver.setOutputRange(-1, 1);
		gearDriver.setAbsoluteTolerance(.1);
		gearDriver.setContinuous(true);
		gearDriver.enable();
		encoders = new Encoders();
		LiveWindow.addActuator("Gear Driver", "PID", gearDriver);
        defineAutonomousActions();
        autoRoutines.put(easy, easyGear);
        autoRoutines.put(left, leftGear);
        autoRoutines.put(right, rightGear);
        autoRoutines.put(forward8, driveSlowlyForward);
        autoRoutines.put(forward15, driveForward);
        autoRoutines.put(bonus, noCamAuto);
        chooser.addDefault("Logan Chaser", easy);
        chooser.addObject("RightGear", right);
        chooser.addObject("Left Gear", left);  
        chooser.addObject("Drive Forward for 8 seconds", forward8);
        chooser.addObject("Drive Forward for 15 seconds", forward15);
        chooser.addObject("Camera-Free Auto!", bonus);
        SmartDashboard.putData("Autonomous Chooser", chooser);
    }
    public void defineAutonomousActions(){
        actions.put("Drive Forward", () -> drive.drive(Math.sqrt(0.5), 0, 0, false));
        actions.put("Drive Forward Slowly", () -> drive.drive(Math.sqrt(0.25), 0, 0, false));
        actions.put("Drive Backwards", () -> drive.drive(-Math.sqrt(0.5), 0, 0, false));
        actions.put("Stop Fast", () -> drive.drive(-0.2, 0, 0, false));
        actions.put("Stop", () -> drive.drive(0, 0, 0, false));
        actions.put("Evict", () -> { evictor.evict(); });
        actions.put("Hold", () -> { evictor.hold(); });
        actions.put("Drive Straight", () -> {
    		drive.drive(Math.sqrt(0.25), 0, drive.getTurnPower(), false);
        });
        endActions.put("Drive Straight with Encoder", () -> {
        	drive.mecanum(drive.encoderOutput, 0, drive.getTurnPower());
        	return drive.encoderController.getError() < 2;
        });
        startActions.put("Set Target Turn To 60 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() + 50);
        });
        startActions.put("Enable Drive PIDs", () -> {
    		drive.getGyro().reset();
    		drive.startDriveStraight();
        	drive.resetEncoders();
        	drive.startEncoderDrive(-80);
        });
        startActions.put("Set Target Turn To 300 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() - 50);
        });
        startActions.put("Enable Straight Drive", () -> {
    		drive.getGyro().reset();
    		drive.startDriveStraight();
        });
        startActions.put("Set Drive Distance to 104", () -> {
        	drive.resetEncoders();
        	drive.startEncoderDrive(-27);
        });
        startActions.put("Enable Gear Drive", () -> {
        	gearDriver.reset();
        	gearDriver.setSetpoint(0);
        	gearDriver.enable();
        });
        endActions.put("Drive to Distance", () -> {
        	drive.mecanum(drive.encoderOutput, 0, 0);
        	System.out.println("Driving " + drive.encoderController.getError() + " inches!");
        	return Math.abs(drive.encoderController.getError()) < 1;
        });
        endActions.put("Reset PID", () -> {
            drive.resetPIDs();
            System.out.println("Got it!");	
            return true;
        });
        endActions.put("Turn To Target Angle", () -> {
            drive.drive(0, 0, drive.getTurnPower(), false);
            System.out.println("Turnin'! " + drive.getActiveController().getError());
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
            return targetFinder.getDistance() < 50 && targetFinder.getDistance() != -1;
        });
        chaseLogan = new AutonomousRoutine(){
        	@Override public void init(){
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(150, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(100, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
        	}
        };
        easyGear = new AutonomousRoutine(){
            @Override public void init(){
            	addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(1500, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(500, () ->  actions.get("Evict").run());
                addSection(40, actions.get("Hold"));
                addSection(310000, actions.get("Stop"));
            }
        };
        noCamAuto = new AutonomousRoutine(){
        	@Override public void init(){
        		addSection(endActions.get("Drive Straight with Encoder"), startActions.get("Enable Drive PIDs"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(500, () ->  actions.get("Evict").run());
                addSection(40, actions.get("Hold"));
                addSection(310000, actions.get("Stop"));
        	}
        };
        leftGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(endActions.get("Drive to Distance"), startActions.get("Set Drive Distance to 104"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 60 Degrees"));
                addSection(endActions.get("Reset PID"));
            	addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(1700, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () ->  actions.get("Evict").run());
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 300 Degrees"));
                addSection(10000, actions.get("Drive Forward Slowly"));
                addSection(310000, actions.get("Stop"));
            }
        };
        rightGear = new AutonomousRoutine(){
            @Override public void init(){
                addSection(endActions.get("Drive to Distance"), startActions.get("Set Drive Distance to 104"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 300 Degrees"));
                addSection(endActions.get("Reset PID"));
            	addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(1700, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () ->  actions.get("Evict").run());
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 60 Degrees"));
                addSection(10000, actions.get("Drive Forward Slowly"));
                addSection(310000, actions.get("Stop"));
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
    	if(false){
    		double dis = 80;
    		Constants.CAMERA_HORIZONTAL_ANGLE = CameraConfig.getXAngle(targetFinder.tt.c.rawXAngle, dis, Constants.CAMERA_HORIZONTAL_OFFSET);
    		Constants.CAMERA_VERTICAL_ANGLE = CameraConfig.getYAngle(targetFinder.tt.c.rawVAngle, Constants.HIGH_GOAL_HEIGHT - Constants.CAMERA_ELEVATION, dis);
    		System.out.println(Constants.CAMERA_HORIZONTAL_ANGLE * 180.0 / Math.PI);
    		System.out.println(Constants.CAMERA_VERTICAL_ANGLE * 180.0 / Math.PI);
    	}
    	selected = autoRoutines.get(chooser.getSelected());
    	selected.begin();
    }
    @Override public void autonomousPeriodic(){
    	selected.tick();
    	//if(t.get() <= 6.5)
    	//	drive.mecanum(0.3, 0, 0);
    	//else drive.mecanum(0, 0, 0);
    }
    @Override public void teleopInit(){
    	drive.setTurnToAngle(60);
    	autoRoutines.get(chooser.getSelected()).stop();
    	imageProcessing.stop();
    	drive.resetEncoders();
    }
    @Override public void testPeriodic(){
    	teleopPeriodic();
    }
    @Override public void teleopPeriodic(){
        outputData();
        
        drive.drive(im.getDrive(), im.getStrafe(), im.getTurn(), im.getShift());
        
        if(im.getIntake())
            intake.motorIn();
        else 
            intake.motorOff();
        
        if(im.getClimb())
            climber.climb();
        else 
            climber.stop();
        
        if(im.getEvict())
            evictor.evict();
        else 
            evictor.hold();
        
        if(im.getShoot())
        	flyWheel.shoot();
        else 
        	flyWheel.stop();
        
        if(im.getTestShoot())
        	if(!flyWheel.isShooting){
        		flyWheel.setRPM(-4000);
        	}
        	else {
        		flyWheel.shoot();
        		SmartDashboard.putNumber("PID Shoot", flyWheel.pidOutput);
        	}
        else 
        	flyWheel.resetPID();
        
        if(im.getTurnLeft()){
        	if(!drive.turnToAngle){
        		drive.getGyro().reset();
        		drive.setTurnToAngle(60);
        	}
        	else {
        		drive.mecanum(0, 0, drive.getTurnPower());
        	}
        }
        if(im.getTurnRight()){
        	if(!drive.turnToAngle){
        		drive.getGyro().reset();
        		drive.setTurnToAngle(-60);
        	}
        	else {
        		drive.mecanum(0, 0, drive.getTurnPower());
        	}
        }
        if(im.getDriveStraight()){
        	if(!drive.driveStraight){
        		drive.getGyro().reset();
        		drive.startDriveStraight();
        	}
        	else {
        		drive.drive(im.getDrive(), im.getStrafe(), drive.getTurnPower(), im.getShift());
        	}
        }
        if(!im.getTurnLeft() && !im.getTurnRight() && !im.getDriveStraight())
        	drive.resetPIDs();
        
        if(im.getGearDrive() && gearDrive){
			if(targetFinder.getTargetType() != CameraTurning.TargetType.GEAR)
				targetFinder.setTargetType(CameraTurning.TargetType.GEAR);
			if(!gearDriver.isEnabled())
				gearDriver.enable();
			drive.mecanum(im.getDrive(), 0, gearDriver.get());
		}
        if(im.getGearDrive() && !gearDrive){
        	gearDriver.reset();
        }
        gearDrive = im.getGearDrive();
        
        if(im.getTestEncoderDrive()){
        	if(!drive.encoderDrive){
        		drive.resetEncoders();
        		drive.startEncoderDrive(-104);
        	}
        	else {
        		drive.mecanum(drive.encoderOutput, 0, 0);
                SmartDashboard.putNumber("Encoder distance traveled", (drive.getRightDistance() - drive.getLeftDistance())/2);
        	}
        }
        else {
        	drive.encoderDrive = false;
        	drive.encoderController.disable();
        }
    }
    private void outputData(){
        navX.outputData();
        //drive.outputPIDData();
        targetFinder.outputData();
        flyWheel.outputData();
        SmartDashboard.putNumber("Angle Error", drive.turnToAngleController.getError());
        SmartDashboard.putNumber("Gear Drive Output", gearDriveOutput);
        SmartDashboard.putNumber("NavX Angle:" , drive.getGyroAngle());
        SmartDashboard.putNumber("Gear Drive Speed", targetFinder.getDrivePower());
        //SmartDashboard.putNumber("NEW ANGLE", targetFinder.tt.c.getTargetAngle());
        SmartDashboard.putNumber("Config angle: ", CameraConfig.getYAngle(targetFinder.tt.c.rawVAngle, 6, 72));
    }
}
