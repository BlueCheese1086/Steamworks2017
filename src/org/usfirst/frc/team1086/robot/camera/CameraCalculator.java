package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.stream.Collectors;
import org.opencv.core.MatOfPoint;

public abstract class CameraCalculator implements PIDSource, CVDataHandler {
    ArrayList<Sighting> visionObjects = new ArrayList();
    double distance, angle;
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
            double avgH = visionObjects.stream().mapToDouble(p -> (p.height) / (visionObjects.size())).sum();
            double angleFromCameraToTarget = getYAngle(midY);
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
        visionObjects = new ArrayList(visionObjects.stream().sorted((a, b) -> { return a.area > b.area ? 1 : a.area < b.area ? -1 : 0; })
                .limit(2).sorted((a, b) -> { return a.x > b.x ? 1 : b.x > a.x ? -1 : 0; }).collect(Collectors.toList()));
        double horizontalAngle = Math.PI / 2 - getXAngle(visionObjects.get(1).x);
        double f = Math.sqrt(distance * distance + Math.pow(Constants.CAMERA_HORIZONTAL_OFFSET, 2)
                                        - 2 * distance * Constants.CAMERA_HORIZONTAL_OFFSET * Math.cos(horizontalAngle));
        double g = Math.asin(Constants.CAMERA_HORIZONTAL_OFFSET * Math.sin(horizontalAngle) / f);
        double b = Math.PI - horizontalAngle - g;
        double angle1 = (Math.PI / 2 - b) - Constants.CAMERA_HORIZONTAL_ANGLE;
        double theta = angle - angle1;
        double h = distance;
        double c = 3;//Half the distance between the pieces of retroreflective tape
        double phi1 = Math.PI - theta - Math.asin(h / c * Math.sin(theta));
        double phi2 = -phi1 + Math.PI - 2 * theta;
        double targetAngle;
        if(visionObjects.get(0).width > visionObjects.get(1).width){
            targetAngle = Math.max(phi1, phi2);
        } else targetAngle = Math.min(phi1, phi2);
        return (Math.PI / 2 - targetAngle) * 180.0 / Math.PI;
    }
    @Override public double pidGet(){
        return angle;
    }
    @Override public void handle(ArrayList<MatOfPoint> m){
        updateObjects(new ArrayList(m.stream().map(n -> new Sighting(n)).collect(Collectors.toList())));
    }
}
