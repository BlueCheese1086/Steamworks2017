package org.usfirst.frc.team1086.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;

public class DriveEncoder {
    public Encoder encoder;
    public static Wheel current = Wheel.MECANUM;
    double mecWheel = 0;
    double colWheel = 0;
//    Direction dir = Direction.FORWARD;
    public DriveEncoder(int num1, int num2, boolean bool, Encoder.EncodingType type){
        encoder = new Encoder(num1, num2, bool, type);
    }
    public void reset(){
        mecWheel = 0;
        colWheel = 0;
        encoder.reset();
    }
    public double getDistance(){
        switchWheel(current);
        return mecWheel / 1024 * Wheel.MECANUM.circumference + colWheel / 1024 * Wheel.COLSON.circumference;
    }
//    public void tick(){
//        if(encoder.g)
//    }
//    public void switchDir(Direction dir){
//        if(current == Wheel.COLSON)
//            colWheel += encoder.get() * this.dir.dir;
//        else mecWheel += encoder.get()* this.dir.dir;
//        this.dir = dir;
//    }
    public void switchWheel(Wheel w){
        if(current == Wheel.COLSON)
            colWheel += encoder.get();// * dir.dir;
        else mecWheel += encoder.get();// * dir.dir;
        encoder.reset();
        current = w;
    }
    public static enum Wheel {
        COLSON(1.0), 
        MECANUM(2.0);
        final double radius, circumference;
        Wheel(double radius){
            this.radius = radius;
            this.circumference = radius * Math.PI * 2;
        }
    }
//    public static enum Direction {
//        FORWARD(1),
//        BACKWARD(-1);
//        final double dir;
//        Direction(double dir){
//            this.dir = dir;
//        }
//    }
}
