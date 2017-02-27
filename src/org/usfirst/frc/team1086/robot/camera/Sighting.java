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
    	return true;
        //if(solidity < 0.8)
        //    return false;
        //return aspectRatio >= 0;
    }
    public boolean possibleGear(){
    	return true;
        //if(solidity < 0.8)
        //    return false;
        //return aspectRatio <= 0.5;
    }
    public Sighting(MatOfPoint m){
        height = Imgproc.boundingRect(m).height;
        width = Imgproc.boundingRect(m).width;
        x = Imgproc.boundingRect(m).x;
        y = Imgproc.boundingRect(m).y;
        area = Imgproc.contourArea(m);
        solidity = area / (width * height);
        aspectRatio = width / height;
        centerX = x + width / 2;
        centerY = y + height / 2;
    }
    @Override public String toString(){
    	return "(" + x + ", " + y + ") to (" + (x + width) + ", " + (y + height) + ")";
    }
}
