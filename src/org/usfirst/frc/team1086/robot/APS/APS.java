package org.usfirst.frc.team1086.robot.APS;

import org.usfirst.frc.team1086.robot.subsystems.*;
import org.usfirst.frc.team1086.robot.camera.*;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class APS {
	Encoders enc = new Encoders();
	GearGoalFinder gg;
	AHRS g;
	double x, y;
	long lastTime = System.currentTimeMillis();
	ReferencePoint[] points = new ReferencePoint[6];
	public APS(){
		gg = new GearGoalFinder();
        if(g == null){
            try {
                g = new AHRS(SerialPort.Port.kMXP);
            } catch(Exception e){e.printStackTrace();}
        }
	}
	public void tick(){
		double ang = g.getYaw();
		double dx = 0;
		double dy = 0;
		dx += ang * Math.cos(enc.pidGet());
		dy += ang * Math.sin(enc.pidGet());
		enc.reset();
		double deltaT = (System.currentTimeMillis() - lastTime) / 1000;
		dx += g.getVelocityX() * deltaT * 0.2;
		dy += g.getVelocityY() * deltaT * 0.2;
		dx /= 1.2;
		dy /= 1.2;
		x += dx;
		y += dy;
		if(gg.estimationIsGood()){
			double angle = gg.getTargetAngle();
			double distance = gg.distance;
			ReferencePoint bestRef = null;
			double estimatedX = -1000;
			double estimatedY = -1000;
			double bestDis = 10000;
			for(ReferencePoint rp : points){
				double theta = rp.theta + angle;
				double thisX = rp.x + distance * Math.cos(theta * Math.PI / 180);
				double thisY = rp.y + distance * Math.sin(theta * Math.PI / 180);
				if(Math.sqrt(Math.pow(estimatedX - thisX, 2) + Math.pow(estimatedY - thisY, 2)) < bestDis){
					estimatedX = thisX;
					estimatedY = thisY;
					bestRef = rp;
				}
			}
			double theta = bestRef.theta + angle;
			x = bestRef.x + distance * Math.cos(theta);
			y = bestRef.y + distance * Math.sin(theta);
		}
		SmartDashboard.putNumber("APS X", x);
		SmartDashboard.putNumber("APS Y", y);
	}
}