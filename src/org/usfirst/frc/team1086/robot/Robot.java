package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.AnalogInput;
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

public class Robot extends IterativeRobot {
    Drivetrain drive;
    InputManager im;
    GearHolder evictor;
    HashMap<String, Section> actions = new HashMap();
    HashMap<String, Section> startActions = new HashMap();
    HashMap<String, Action> endActions = new HashMap();
    AutonomousRoutine easyGear;
    AutonomousRoutine leftGear;
    AutonomousRoutine rightGear;
    AutonomousRoutine chaseLogan;
    AutonomousRoutine driveSlowlyForward, driveForward, noCamAuto;
    AutonomousRoutine leftGearShoot, rightGearShoot;
    AutonomousRoutine centerShootRight, centerShootLeft;
    AutonomousRoutine selected;
    CameraTurning targetFinder;
    ShooterTest flyWheel;
    Ultrasonic ultra;
    ImageProcessing imageProcessing;
    Intake intake;
    Climber climber;
    Agitator agitator;
    Compressor compressor;
    Gyro navX;
    PIDController gearDriver, ultraDriver;
    double gearDriveOutput;
    SendableChooser<AutonomousRoutine> chooser = new SendableChooser<>();
    SendableChooser<Boolean> config = new SendableChooser<>();
    Timer t;
    Encoders encoders;
    boolean buttonDown = false;
    boolean backward = false;
    boolean gearDrive = false;
    @Override public void robotInit(){
        config.addDefault("Do not config", false);
        config.addObject("Configure Camera (RUN LOGAN CHASER)", true);
        drive = new Drivetrain();
        drive.resetEncoders();
        evictor = new GearHolder();
        im = new InputManager();
        targetFinder = new CameraTurning();
        ultra = new Ultrasonic(0);
        flyWheel = new ShooterTest();
        intake = new Intake();
        t = new Timer();
        climber = new Climber();
        agitator = new Agitator();
        imageProcessing = new ImageProcessing();
        imageProcessing.setCameraTarget(targetFinder);
        imageProcessing.start();
        navX = new Gyro();
        compressor = new Compressor(RobotMap.COMPRESSOR);
        compressor.setClosedLoopControl(true);
        navX.reset();
        System.out.println("The calculator is " + targetFinder.calculator);
        gearDriver = new PIDController(-0.025, 0, -.0583, 0, targetFinder.calculator, v -> gearDriveOutput = v);		
        gearDriver.setInputRange(-180.0, 180.0);
		gearDriver.setOutputRange(-1, 1);
		gearDriver.setAbsoluteTolerance(.1);
		gearDriver.setContinuous(true);
		gearDriver.enable();
		gearDriver.setSetpoint(0);
		ultraDriver = new PIDController(-0.11, 0, -0.1, 0, ultra, v -> {});
		ultraDriver.setInputRange(0, 240);
		ultraDriver.setOutputRange(-0.45, 0.45);
		ultraDriver.setAbsoluteTolerance(0.5);
		ultraDriver.setContinuous(false);
		ultraDriver.enable();
		encoders = new Encoders();
		LiveWindow.addActuator("Gear Driver", "PID", gearDriver);
        defineAutonomousActions();
        chooser.addDefault("Logan Chaser", easyGear);
        chooser.addObject("RightGear", rightGear);
        chooser.addObject("Left Gear", leftGear);  
        chooser.addObject("Drive Forward for 8 seconds", driveSlowlyForward);
        chooser.addObject("Drive Forward for 15 seconds", driveForward);
        chooser.addObject("Camera-Free Auto!", noCamAuto);
        chooser.addObject("Right Gear then Shoot!", rightGearShoot);
        chooser.addObject("Left Gear then Shoot!", leftGearShoot);
        chooser.addObject("Center Gear then Shoot Right", centerShootRight);
        chooser.addObject("Center Gear then Shoot Left", centerShootLeft);
        SmartDashboard.putData("Autonomous Chooser", chooser);
        SmartDashboard.putData("Auto Config", config);
    }
    public void defineAutonomousActions(){
        actions.put("Drive Forward", () -> drive.drive(Math.sqrt(0.5), 0, 0, false));
        actions.put("Drive Forward Slowly", () -> drive.drive(Math.sqrt(0.25), 0, 0, false));
        actions.put("Drive Backwards", () -> drive.drive(-Math.sqrt(0.5), 0, 0, false));
        actions.put("Stop Fast", () -> drive.drive(-0.2, 0, 0, false));
        actions.put("Stop", () -> drive.drive(0, 0, 0, false));
        actions.put("Evict", () -> evictor.evict());
        actions.put("Hold", () -> evictor.hold());
        //actions.put("Shoot!", () -> flyWheel.shoot());
        actions.put("Run Agitator", () -> agitator.agitate());
        actions.put("Stop Agitator", () -> agitator.stop());
        actions.put("Drive Straight", () -> {
        	System.out.println("Driving straight - Error: " + drive.getActiveController().getError() + " Power: " + drive.getTurnPower());
    		drive.drive(Math.sqrt(0.25), 0, drive.getTurnPower(), false);
        });
        endActions.put("Drive Straight with Encoder", () -> {
        	System.out.println("Driving to boiler: " + drive.encoderController.getError());
        	drive.mecanum(drive.encoderController.get(), 0, drive.getTurnPower());
        	System.out.println("Encoder Output: " + drive.encoderController.get());
        	return Math.abs(drive.encoderController.getError()) < 2.5;
        });
        startActions.put("Start Shooter", () -> {
        	//flyWheel.setRPM(-3500);
        });
        startActions.put("Set Target Turn To 60 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() + 50);
        });
        startActions.put("Start Drive to Wall", () -> {
        	ultraDriver.reset();
        	ultraDriver.enable();
        	ultraDriver.setSetpoint(7.0);
        });
        startActions.put("Enable Drive PIDs", () -> {
    		drive.getGyro().reset();
    		drive.startDriveStraight();
        	drive.resetEncoders();
        	drive.startEncoderDrive(-80);
        });
        startActions.put("Enable Drive to Boiler", () -> {
        	drive.startDriveStraight();
        	drive.encoderDrive = false;
        	drive.resetEncoders();
        	drive.startEncoderDrive(39);
        	System.out.println("Starting Enc Error: " + drive.encoderController.getError());
        });
        startActions.put("Set Target Turn To 300 Degrees", () -> {
            drive.setTurnToAngle(drive.getGyro().getAngle() - 50);
        });
        startActions.put("Set Turn to 135 Absolute", () -> {
        	drive.setTurnToAngle(-44);
        });
        startActions.put("Set Turn to -135 Absolute", () -> {
        	drive.setTurnToAngle(44);
        });
        startActions.put("Set Turn to 90 Absolute", () -> {
        	drive.setTurnToAngle(90);
        });
        startActions.put("Set Turn to -90 Absolute", () -> {
        	drive.setTurnToAngle(-90);
        });
        startActions.put("Enable Straight Drive", () -> {
        	System.out.println("Straight");
    		drive.startDriveStraight();
        });
        startActions.put("Set Drive Distance to 104", () -> {
        	drive.resetEncoders();
        	drive.startEncoderDrive(-23);
        });
        startActions.put("Set Drive Distance to 35", () -> {
        	drive.resetEncoders();
        	drive.startEncoderDrive(35);
        });
        startActions.put("Set Drive Distance to 144", () -> {
        	drive.resetEncoders();
        	drive.startEncoderDrive(72);
        });
        startActions.put("Enable Gear Drive", () -> {
        	gearDriver.reset();
        	gearDriver.setSetpoint(0);
        	gearDriver.enable();
        });
        endActions.put("Drive to Wall", () -> {
        	drive.mecanum(ultraDriver.get(), 0, 0);
        	System.out.println("Driving " + ultraDriver.getError() + " inches!");
        	return Math.abs(ultraDriver.getError()) < 1;
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
            return drive.getActiveController().onTarget() && drive.getActiveController().getAvgError() < 2;
        });
        endActions.put("Drive to Sight", () -> {
			drive.mecanum(0.4, 0, 0);
			return targetFinder.getDistance() != -1 && targetFinder.getDistance() < 70;
		});
        endActions.put("Chase Logan", () -> {
        	gearDriver.enable();
            double drivePower = targetFinder.getDrivePower();           
            drive.drive(Math.signum(drivePower) * Math.sqrt(Math.abs(drivePower)), 0.0, gearDriveOutput, false);
            System.out.println("Target Distance: " + targetFinder.getDistance());
            System.out.println("Target Angle: " + targetFinder.getAngle());
            System.out.println("Gear Driver Angle: " + gearDriver.getError() + " - Power: " + gearDriveOutput);
            SmartDashboard.putNumber("Gear Drive Angle", gearDriver.getError());
            SmartDashboard.putNumber("Gear Drive Turn", gearDriveOutput);
            return targetFinder.getDistance() < 55 && targetFinder.getDistance() != -1;
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
                addSection(500, () ->  actions.get("Evict").run());
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(310000, actions.get("Stop"));
            }
        };
        /*
        easyGear = new AutonomousRoutine(){
            @Override public void init(){
            	addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                //addSection(1500, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
        		addSection(endActions.get("Drive to Wall"), startActions.get("Start Drive to Wall"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () ->  actions.get("Evict").run());
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(310000, actions.get("Stop"));
            }
        };*/
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
        centerShootLeft = new AutonomousRoutine(){
        	@Override public void init(){
        		addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(1500, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () ->  actions.get("Evict").run());
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Turn to -90 Absolute"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Drive to Distance"), startActions.get("Set Drive Distance to 144"));
                addSection(endActions.get("Reset PID"), startActions.get("Start Shooter"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Turn to -135 Absolute"));
                addSection(endActions.get("Reset PID"));
                addSection(300, actions.get("Drive Backwards"), startActions.get("Enable Straight Drive"));
                addSection(40, actions.get("Stop"));
                addSection(100000, () -> { actions.get("Shoot!").run(); actions.get("Run Agitator").run(); }, startActions.get("Start Shooter"));
        	}
        };
        centerShootRight = new AutonomousRoutine(){
        	@Override public void init(){
        		addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(1500, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () ->  actions.get("Evict").run());
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Turn to 90 Absolute"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Drive to Distance"), startActions.get("Set Drive Distance to 144"));
                addSection(endActions.get("Reset PID"), startActions.get("Start Shooter"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Turn to 135 Absolute"));
                addSection(endActions.get("Reset PID"));
                addSection(300, actions.get("Drive Backwards"), startActions.get("Enable Straight Drive"));
                addSection(40, actions.get("Stop"));
                addSection(100000, () -> { actions.get("Shoot!").run(); actions.get("Run Agitator").run(); }, startActions.get("Start Shooter"));
        	}
        };
        leftGearShoot = new AutonomousRoutine(){
            @Override public void init(){
                addSection(endActions.get("Drive to Distance"), startActions.get("Set Drive Distance to 104"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 60 Degrees"));
                addSection(endActions.get("Reset PID"));
            	addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(1900, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(200, actions.get("Stop"));
                addSection(250, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(endActions.get("Reset PID"));
                addSection(500, actions.get("Drive Backwards"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Turn to -135 Absolute"));
                addSection(endActions.get("Reset PID"), startActions.get("Start Shooter"));
                addSection(3000, () -> { actions.get("Shoot!").run(); return endActions.get("Drive Straight with Encoder").run(); }, startActions.get("Enable Drive to Boiler"));
                addSection(40, actions.get("Stop"));
                addSection(100000, () -> { actions.get("Shoot!").run(); actions.get("Run Agitator").run(); }, startActions.get("Start Shooter"));
            }
        };
        rightGearShoot = new AutonomousRoutine(){
            @Override public void init(){
                addSection(endActions.get("Drive to Distance"), startActions.get("Set Drive Distance to 104"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 300 Degrees"));
                addSection(endActions.get("Reset PID"));
            	addSection(endActions.get("Drive to Sight"));
        		addSection(endActions.get("Chase Logan"), startActions.get("Enable Gear Drive"));
                addSection(1900, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(200, actions.get("Stop"));               
                addSection(250, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(endActions.get("Reset PID"));
                addSection(500, actions.get("Drive Backwards"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Turn to 135 Absolute"));
                addSection(endActions.get("Reset PID"), startActions.get("Start Shooter"));
                addSection(3000, () -> { actions.get("Shoot!").run(); return endActions.get("Drive Straight with Encoder").run(); }, startActions.get("Enable Drive to Boiler"));
                addSection(40, actions.get("Stop"));
                addSection(100000, () -> { actions.get("Shoot!").run(); actions.get("Run Agitator").run(); }, startActions.get("Start Shooter"));
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
                addSection(1900, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
                addSection(500, () -> { actions.get("Drive Backwards").run(); actions.get("Evict").run(); });
                addSection(40, actions.get("Hold"));
                addSection(endActions.get("Reset PID"));
                addSection(endActions.get("Turn To Target Angle"), startActions.get("Set Target Turn To 60 Degrees"));
                addSection(10000, actions.get("Drive Forward Slowly"));
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
                addSection(1900, actions.get("Drive Straight"), startActions.get("Enable Straight Drive"));
                addSection(300, actions.get("Stop Fast"));
                addSection(500, actions.get("Stop"));
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
    	if(chooser.getSelected() == easyGear || chooser.getSelected() == centerShootLeft || chooser.getSelected() == centerShootRight){
    		System.out.println("-------------------Configuring!---------------------------");
    		double dis = 80;//ultra.<get distance>
    		Constants.CAMERA_HORIZONTAL_ANGLE = CameraConfig.getXAngle(targetFinder.calculator.rawXAngle, dis, Constants.CAMERA_HORIZONTAL_OFFSET);
    		Constants.CAMERA_VERTICAL_ANGLE = CameraConfig.getYAngle(targetFinder.calculator.rawVAngle, Constants.GOAL_HEIGHT - Constants.CAMERA_ELEVATION, dis);
    		System.out.println(Constants.CAMERA_HORIZONTAL_ANGLE * 180.0 / Math.PI);
    		System.out.println(Constants.CAMERA_VERTICAL_ANGLE * 180.0 / Math.PI);
    	}
    	drive.getGyro().navX.reset();
    	selected = chooser.getSelected();
    	selected.begin();
    }
    @Override public void autonomousPeriodic(){
    	selected.tick();
    	outputData();
    }
    @Override public void teleopInit(){
    	drive.setTurnToAngle(60);
    	//selected.stop();
    	drive.resetEncoders();
    	agitator.stop();
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
        
       
        
        
        if(im.getTestTestShoot())
        	flyWheel.shoot();
        else flyWheel.stop();
        
        if(im.getTurnLeft()){
        	if(!drive.turnToAngle){
        		drive.setTurnToAngle(drive.getGyroAngle() + 60);
        	}
        	else {
        		drive.mecanum(0, 0, drive.getTurnPower());
        	}
        }
        if(im.getTurnRight()){
        	if(!drive.turnToAngle){
        		drive.setTurnToAngle(drive.getGyroAngle() - 60);
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
        
        if(im.auxStick.getRawButton(7))
        	agitator.agitate();
        else agitator.stop();
    }
    private void outputData(){
        navX.outputData();
        drive.outputPIDData();
        targetFinder.outputData();
        flyWheel.outputData();
        SmartDashboard.putNumber("Ultrasonic Sensor", ultra.get());
        SmartDashboard.putNumber("Angle Error", drive.turnToAngleController.getError());
        SmartDashboard.putNumber("Gear Drive Output", gearDriveOutput);
        SmartDashboard.putNumber("NavX Angle:" , drive.getGyroAngle());
        SmartDashboard.putNumber("Gear Drive Speed", targetFinder.getDrivePower());
    }
}
