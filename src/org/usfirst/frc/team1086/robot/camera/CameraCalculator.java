package org.usfirst.frc.team1086.robot.camera;

import edu.wpi.first.wpilibj.PIDSource;
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
            double midY = visionObjects.stream().mapToDouble(p -> (p.centerY + p.height / 2) / (visionObjects.size())).sum();
            double avgH = visionObjects.stream().mapToDouble(p -> (p.height) / (visionObjects.size())).sum();
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
    @Override public double pidGet(){
        return angle;
    }
    @Override public void handle(ArrayList<MatOfPoint> m){
        updateObjects(new ArrayList(m.stream().map(n -> new Sighting(n)).collect(Collectors.toList())));
    }
}