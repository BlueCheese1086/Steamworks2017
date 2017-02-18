package org.usfirst.frc.team1086.robot.pixy;

public class PixyPacket {
    public int x, y, width, height, checksumError;
    @Override public String toString(){
        return "x: " + x + " y: " + y + " width: " + width + " height: " + height;
    }
}
