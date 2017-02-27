package org.usfirst.frc.team1086.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
public class Encoders implements PIDSource, PIDOutput {
    DriveEncoder enc1;
    DriveEncoder enc2;
    DriveEncoder enc3;
    DriveEncoder enc4;
    PIDController drivePID;
    public Encoders() {
        enc1 = new DriveEncoder(0, 1, false, Encoder.EncodingType.k4X);
        enc2 = new DriveEncoder(0, 1, false, Encoder.EncodingType.k4X);
        enc3 = new DriveEncoder(0, 1, false, Encoder.EncodingType.k4X);
        enc4 = new DriveEncoder(0, 1, false, Encoder.EncodingType.k4X);
        drivePID = new PIDController(0, 0, 0, this, this);
        drivePID.setInputRange(-240, 240);
        drivePID.setOutputRange(-1, 1);
        drivePID.setAbsoluteTolerance(1);
    }
    public void enablePID(double distance){
        enc1.reset();
        enc2.reset();
        enc3.reset();
        enc4.reset();
        drivePID.reset();
        drivePID.setSetpoint(distance);
        drivePID.enable();
    }
    public double getPID(){
        return drivePID.get();
    }
    @Override public void setPIDSourceType(PIDSourceType pidSource){}
    @Override public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }
    @Override public double pidGet(){
        return ((enc1.getDistance() + enc2.getDistance() + enc3.getDistance() + enc4.getDistance()) / 4);
    }
    @Override public void pidWrite(double output){}
}