package org.usfirst.frc.team1086.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
public class Encoders implements PIDSource, PIDOutput {
    DriveEncoder leftEnc;
    DriveEncoder rightEnc;
    PIDController drivePID;
    public Encoders() {
        leftEnc = new DriveEncoder(1, 2, false, Encoder.EncodingType.k4X);
        rightEnc = new DriveEncoder(3, 4, false, Encoder.EncodingType.k4X);
        drivePID = new PIDController(0, 0, 0, this, this);
        drivePID.setInputRange(-240, 240);
        drivePID.setOutputRange(-1, 1);
        drivePID.setAbsoluteTolerance(1);
    }
    public void reset(){
    	leftEnc.reset();
    	rightEnc.reset();
    }
    public void enablePID(double distance){
        leftEnc.reset();
        rightEnc.reset();
        drivePID.reset();
        drivePID.setSetpoint(distance);
        drivePID.enable();
    }
    public double getDistance(){
    	return ((leftEnc.getDistance() + rightEnc.getDistance()) / 2);
    }
    public double getPID(){
        return drivePID.get();
    }
    @Override public void setPIDSourceType(PIDSourceType pidSource){}
    @Override public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }
    @Override public double pidGet(){
        return ((leftEnc.getDistance() + rightEnc.getDistance()) / 2);
    }
    @Override public void pidWrite(double output){}
}