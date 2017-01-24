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
        return false;
    }
    public boolean possibleGear(){
        return false;
    }
    public Sighting(MatOfPoint m){
        height=m.height();
        width=m.width();
        x = Imgproc.boundingRect(m).x;
        y = Imgproc.boundingRect(m).y;
        area = Imgproc.contourArea(m);
        solidity = area / (width * height);
        aspectRatio= width / height;
        centerX = x+width/2;
        centerY = y+height/2;
    }
}
