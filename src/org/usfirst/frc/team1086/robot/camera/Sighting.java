package org.usfirst.frc.team1086.robot.camera;

import org.opencv.core.MatOfPoint;

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
        
    }
}
