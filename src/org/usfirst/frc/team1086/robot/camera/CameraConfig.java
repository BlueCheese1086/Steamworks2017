package org.usfirst.frc.team1086.robot.camera;
public class CameraConfig {
    //Distance is distance from camera to base of target for all methods
   //public static double getXAngle(double rawA, double dis, double cameraOffset){
        //rawA is angle from camera to target. cameraOffset is distance from camera to center of robot
        //ROBOT MUST BE DIRECTLY FACING THE TARGET. If the face of the target forms a line and the center of the robot makes a line, the two lines MUST be perpendicular.
        //Robot must also be directly in front of the target.
    //    return Math.asin(Math.sqrt(dis * dis - cameraOffset * cameraOffset) / dis) - rawA;
    //}
    public static double getXAngle(double rawA, double dis, double cameraOffset){
    	return Math.acos(cameraOffset / dis) - rawA;
    }
    public static double getYAngle(double rawA, double dHeight, double dis){
        //dHeight is 83 - camera's height off of the ground. rawA is the vertical angle from camera to target. 
        return Math.atan2(dHeight, dis) - rawA;
    }
    public static double getCameraHeight(double vA, double dis){
        //vA is the sum of the vertical angle from camera to target + the camera's vertical angle.
        double tHeight = 83.0;
        return tHeight - Math.tan(vA) * dis;
    }
    public static double getCameraOffset(double ang, double dis){
        //ang is angle from camera to center of target.
        //ROBOT MUST BE DIRECTLY FACING THE TARGET. If the face of the target forms a line and the center of the robot makes a line, the two lines MUST be perpendicular.
        //Robot must also be directly in front of the target.
        return dis * Math.cos(ang);
    }
}
