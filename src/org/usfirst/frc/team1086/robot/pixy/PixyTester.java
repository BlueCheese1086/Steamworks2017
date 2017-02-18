package org.usfirst.frc.team1086.robot.pixy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyTester {
    public PixyI2C pixy;
    public PixyPacket[] packets = new PixyPacket[7];
    public PixyTester(){
        pixy = new PixyI2C(packets);
    }
    
    public void run(){
        new Thread(() -> {
            for(int i = 0; i < packets.length; i++){
                packets[i] = null;
            }
            for(int i = 1; i < 8; i++){
                try {
                    packets[i - 1] = pixy.readPacket(i);
                } catch(Exception e){System.out.println("Couldn't read signature" + i);}
                if(packets[i - 1] == null)
                    continue;
                SmartDashboard.putString("Pixy Data " + i, packets[i - 1].toString());
            }
        }).start();
    }
}
