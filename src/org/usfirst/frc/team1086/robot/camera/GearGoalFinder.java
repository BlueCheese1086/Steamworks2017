package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDSourceType;
import java.util.ArrayList;
import java.util.stream.Collectors;

public class GearGoalFinder extends CameraCalculator {
    public static final double LIFT_LENGTH = 14;
    public GearGoalFinder(){
        super(Constants.HIGH_GOAL_HEIGHT);
    }/*
    @Override public void calculateAngle(){
    	super.calculateAngle();
    	double radianAngle = angle * Math.PI / 180;
    	double radianNewAngle = Math.atan2(distance * Math.cos(radianAngle) - LIFT_LENGTH, distance * Math.sin(radianAngle));
    	double newDistance = distance * Math.sin(radianAngle) / Math.cos(radianNewAngle);
    	distance = newDistance;
    	angle = radianNewAngle * 180.0 / Math.PI;
    }*/
    @Override public void validateTargets(){
        //visionObjects = new ArrayList(visionObjects.stream().filter(s -> s.solidity > 0.8 && s.aspectRatio < 1.5).collect(Collectors.toList()));
    }
    @Override public boolean estimationIsGood(){
    	//return true;
        return visionObjects.size() >= 2 && distance >= 0 && distance <= 240 && Math.abs(angle) < 90;
    }
    @Override public void setPIDSourceType(PIDSourceType pidSource){
    }
    @Override public PIDSourceType getPIDSourceType(){
        return PIDSourceType.kDisplacement;
    }
}