
package org.usfirst.frc.team1086.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Gyro implements PIDSource {
    private static AHRS navX;
    public Gyro(){
        if(navX == null){
            try {
                navX = new AHRS(SerialPort.Port.kMXP);
                System.out.println("Hey, I just instantiated the navx! Enjoy!");
            } catch(Exception e){e.printStackTrace();}
        }
    }   
    public double getAngle(){
        return navX.getYaw();
    }    
    public void reset(){
        navX.reset();
    }
    public double getNormalizedAngle(){
        return normalizeAngle(getAngle());
    }
    public double normalizeAngle(double d){
        double ang = (d % 360 + 360 % 360);
        return ang > 180 ? ang - 360 : ang;
    }
    public void outputData(){
        SmartDashboard.putNumber("Angle", getAngle());
    }
    @Override public void setPIDSourceType(PIDSourceType pidSource){}
    @Override public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }
    @Override public double pidGet(){
        return getAngle();
    }
   
}
