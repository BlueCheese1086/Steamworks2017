package org.usfirst.frc.team1086.robot.subsystems;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
public class Encoders implements PIDSource, PIDOutput {
    Encoder enc1;
    Encoder enc2;
    Encoder enc3;
    Encoder enc4;
    Encoder encHood;
    Encoder encShoot;
    PIDController drivePID;
    public Encoders() {
        enc1 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        enc1.setMinRate(10);
        enc1.setDistancePerPulse(5);
        enc1.setReverseDirection(true);
        enc1.setSamplesToAverage(7);
        
        enc2 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        enc2.setMinRate(10);
        enc2.setDistancePerPulse(5);
        enc2.setReverseDirection(true);
        enc2.setSamplesToAverage(7);
        
        enc3 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        enc3.setMinRate(10);
        enc3.setDistancePerPulse(5);
        enc3.setReverseDirection(true);
        enc3.setSamplesToAverage(7);
    
        enc4 = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        enc4.setMinRate(10);
        enc4.setDistancePerPulse(5);
        enc4.setReverseDirection(true);
        enc4.setSamplesToAverage(7);

        encHood = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        encHood.setMinRate(10);
        encHood.setDistancePerPulse(5);
        encHood.setReverseDirection(true);
        encHood.setSamplesToAverage(7);
        
        encShoot = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
        encShoot.setMinRate(10);
        encShoot.setDistancePerPulse(5);
        encShoot.setReverseDirection(true);
        encShoot.setSamplesToAverage(7);
        
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