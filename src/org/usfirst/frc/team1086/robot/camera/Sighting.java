package org.usfirst.frc.team1086.robot.camera;

import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Imgproc;

public class Sighting {
    int x, y;
    double centerX, centerY;
    int height, width;
    double area;
    double solidity;
    double aspectRatio;
    public boolean possibleBoiler(){
        if(solidity < 0.8)
            return false;
        return aspectRatio >= 2;
    }
    public boolean possibleGear(){
        if(solidity < 0.8)
            return false;
        return aspectRatio <= 0.5;
    }
    public Sighting(MatOfPoint m){
        height = m.height();
        width = m.width();
        x = Imgproc.boundingRect(m).x;
        y = Imgproc.boundingRect(m).y;
        area = Imgproc.contourArea(m);
        solidity = area / (width * height);
        aspectRatio = width / height;
        centerX = x + width / 2;
        centerY = y + height / 2;
    }
}
