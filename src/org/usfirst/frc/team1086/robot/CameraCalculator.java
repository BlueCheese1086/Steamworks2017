package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.PIDSource;
import java.awt.Polygon;
import java.util.ArrayList;

public abstract class CameraCalculator implements PIDSource {
    ArrayList<Polygon> visionObjects = new ArrayList();
    double distance, angle;
    public static double TARGET_HEIGHT;
    public CameraCalculator(double height){
        TARGET_HEIGHT = height;
    }
    public void updateObjects(ArrayList<Polygon> polys){
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
            double midY = visionObjects.stream().mapToDouble(p -> p.getBounds2D().getCenterY() / visionObjects.size()).sum();
            double angleFromCameraToTarget = getYAngle(midY);
            double verticalAngle = angleFromCameraToTarget + Constants.CAMERA_VERTICAL_ANGLE;
            double changeInY = TARGET_HEIGHT - Constants.CAMERA_ELEVATION;
            double distanceToTarget = changeInY / Math.sin(verticalAngle);
            double horizontalDistance = distanceToTarget * Math.cos(verticalAngle);
            distance = horizontalDistance;
        }
    }
    public void calculateAngle(){
        if(visionObjects.isEmpty()){
            angle = Math.PI;
        } else {
            double midX = visionObjects.stream().mapToDouble(p -> p.getBounds2D().getCenterX() / visionObjects.size()).sum();
            double horizontalAngle = Math.PI / 2 - getXAngle(midX);
            double f = Math.sqrt(
                            distance * distance + Math.pow(Constants.CAMERA_HORIZONTAL_OFFSET, 2)
                                            - 2 * distance * Constants.CAMERA_HORIZONTAL_OFFSET * Math.cos(horizontalAngle));
            double c = Math.asin(Constants.CAMERA_HORIZONTAL_OFFSET * Math.sin(horizontalAngle) / f);
            double b = Math.PI - horizontalAngle - c;
            angle = (Math.PI / 2 - b) - Constants.CAMERA_HORIZONTAL_ANGLE;
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
        return Math.atan((cy - y) / VF);
    }
    public double polygonArea(Polygon p){
        return 0;
    }
    @Override public double pidGet(){
        return angle;
    }
}