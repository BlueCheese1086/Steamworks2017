package org.usfirst.frc.team1086.robot.pixy;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PixyI2C {
    I2C pixy;
    Port port = Port.kOnboard;
    PixyPacket[] packets;
    public PixyI2C(PixyPacket[] packets){
        pixy = new I2C(port, 0x54);
        this.packets = packets;
    }   
    public int convert(byte upper, byte lower){
        //Converts the raw data to readable integers
        return (((int)upper & 0xff) << 8) | ((int)lower & 0xff);
    }
    public PixyPacket readPacket(int signature) throws Exception {
        int checksum, sig;
        byte[] rawData = new byte[32];
        try {
            pixy.readOnly(rawData, 32);
        } catch(RuntimeException e){e.printStackTrace();}
        if(rawData.length < 32){
            System.out.println("Did not receive all 32 bytes of data");
            return null;
        }
        for(int i = 0; i <= 16; i++){
            int syncWord = convert(rawData[i+1], rawData[i]); //First two bytes
            if(syncWord == 0xaa55){ //0xaa55 signifies the start of valid data
                syncWord = convert(rawData[i+3], rawData[i+2]); 
                //Actually need 2 of 0xaa55 to be a valid data stream
                if(syncWord != 0xaa55){ //shift back if only got 1 0xaa55
                    i -= 2;
                }//5123
                checksum = convert(rawData[i+5], rawData[i+4]);
                sig = convert(rawData[i+7], rawData[i+6]);
                if(sig <= 0 || sig > packets.length){
                    System.out.println("Invalid sig value");
                    break;
                }
                packets[sig - 1] = new PixyPacket();
                packets[sig - 1].x = convert(rawData[i+9], rawData[i+8]);
                packets[sig - 1].y = convert(rawData[i+11], rawData[i+10]);
                packets[sig - 1].width = convert(rawData[i+13], rawData[i+12]);
                packets[sig - 1].height = convert(rawData[i+15], rawData[i+14]);

                if(checksum != sig + packets[sig - 1].x + packets[sig - 1].y + packets[sig - 1].width + packets[sig - 1].height){
                    System.out.println("THIS BLOCK SHOULD NEVER HAVE BEEN CALLED");
                    packets[sig - 1] = null;
                    throw new Exception();
                }
                break;
            }
        }
        PixyPacket newPacket = packets[signature - 1];
        packets[signature - 1] = null;
        return newPacket;
    }
}