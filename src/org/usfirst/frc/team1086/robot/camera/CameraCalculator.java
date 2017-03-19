package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.stream.Collectors;
import org.opencv.core.MatOfPoint;

public abstract class CameraCalculator implements PIDSource, CVDataHandler {
    ArrayList<Sighting> visionObjects = new ArrayList<>();
    public double distance, angle, rawVAngle;
    public static double TARGET_HEIGHT;
    public CameraCalculator(double height){
        TARGET_HEIGHT = height;
    }
    public void updateObjects(ArrayList<Sighting> polys){
        visionObjects.clear();
        visionObjects.addAll(polys);
        validateTargets();
        calculateDistance();
        calculateAngle();
    }
    public abstract void validateTargets();
    public abstract boolean estimationIsGood();
    public void calculateDistance(){
        if(visionObjects.isEmpty()){
            distance = -1;
        } else {
            double midY = visionObjects.stream().mapToDouble(p -> (p.centerY - p.height / 2) / (visionObjects.size())).sum();
            //System.out.println("Mid Y: " + midY);
            double angleFromCameraToTarget = getYAngle(midY);
            rawVAngle = angleFromCameraToTarget;
            //System.out.println("Angle to Target: " + angleFromCameraToTarget);
            double verticalAngle = angleFromCameraToTarget + Constants.CAMERA_VERTICAL_ANGLE;
            //System.out.println("Vert Angle: " + verticalAngle * 180 / Math.PI);
            double changeInY = TARGET_HEIGHT - Constants.CAMERA_ELEVATION;
            //System.out.println("DELTA Y: " + changeInY);
            double distanceToTarget = changeInY / Math.sin(verticalAngle);
            double horizontalDistance = distanceToTarget * Math.cos(verticalAngle);
            distance = horizontalDistance;
          //  System.out.println("Distance: " + distance);
        }
    }
    public void calculateAngle(){
        if(visionObjects.isEmpty()){
            angle = 180;
        } else {
            double midX = visionObjects.stream().mapToDouble(p -> p.centerX / visionObjects.size()).sum();
            double horizontalAngle = Math.PI / 2 - getXAngle(midX);
            double f = Math.sqrt(
                            distance * distance + Math.pow(Constants.CAMERA_HORIZONTAL_OFFSET, 2)
                                            - 2 * distance * Constants.CAMERA_HORIZONTAL_OFFSET * Math.cos(horizontalAngle));
            double c = Math.asin(Constants.CAMERA_HORIZONTAL_OFFSET * Math.sin(horizontalAngle) / f);
            double b = Math.PI - horizontalAngle - c;
            angle = ((Math.PI / 2 - b) - Constants.CAMERA_HORIZONTAL_ANGLE) * 180.0 / Math.PI;
     //       System.out.println("Angle: " + angle);
        }
    }
    public double getXAngle(double x){
        double HF = (Constants.MAX_X_PIXELS / 2) / Math.tan(Constants.CAMERA_HFOV / 2);
        double cx = (Constants.MAX_X_PIXELS / 2) - 0.5;
        SmartDashboard.putNumber("RAW ANGLE: ", Math.atan((cx - x) / HF));
        return Math.atan((cx - x) / HF);
    }
    public double getYAngle(double y){
        double VF = (Constants.MAX_Y_PIXELS / 2) / Math.tan(Constants.CAMERA_VFOV / 2);
        double cy = (Constants.MAX_Y_PIXELS / 2) - 0.5;
        SmartDashboard.putNumber("Raw Angle", Math.atan((cy - y) / VF) * 180.0 / Math.PI);	
        return Math.atan((cy - y) / VF);
    }
    public double getTargetAngle(){
        if(visionObjects.size() < 2)
            return 0;
        else {
	        visionObjects = new ArrayList<Sighting>(visionObjects.stream().sorted((a, b) -> { return a.area > b.area ? 1 : a.area < b.area ? -1 : 0; })
	                .limit(2).sorted((a, b) -> {return a.x > b.x ? 1 : b.x > a.x ? -1 : 0;}).collect(Collectors.toList()));
	        if(visionObjects.size() != 2)
	        	return 0;
	        double angle1 = -getXAngle(visionObjects.get(0).x + visionObjects.get(0).width);
	        double angle2 = -getXAngle(visionObjects.get(1).x);
	        double distance = 0;
	        if(angle1 > angle2){
	        	distance = distance(visionObjects.get(0).y);
	        } else distance = distance(visionObjects.get(1).y);
	        double theta = Math.abs(angle1 - angle2);
	        double asin = Math.asin(distance * Math.sin(theta) / 6);
	        if(angle2 > angle1)
	        	asin = Math.PI - asin;
	        return asin + theta / 2;
        }
    }
    @Override public double pidGet(){
        return angle;
    }
    @Override public void handle(ArrayList<MatOfPoint> m){
        updateObjects(new ArrayList<Sighting>(m.stream().map(n -> new Sighting(n)).collect(Collectors.toList())));
    }
    public double distance(double h){
        double angleFromCameraToTarget = getYAngle(h);
        double verticalAngle = angleFromCameraToTarget + Constants.CAMERA_VERTICAL_ANGLE;
        double changeInY = TARGET_HEIGHT - Constants.CAMERA_ELEVATION;
        double distanceToTarget = changeInY / Math.sin(verticalAngle);
        double horizontalDistance = distanceToTarget * Math.cos(verticalAngle);
        return horizontalDistance;
    }
    public double acot(double slope){
    	if(slope == 0)
    		return Math.PI / 2;
    	return Math.atan(1/slope);
    }
}
