package org.usfirst.frc.team1086.robot;
public class ASyncDelayedTask extends Thread {
    int time;
    Runnable task;
    public ASyncDelayedTask(int time, Runnable task){
        this.time = time;
        this.task = task;
    }
    @Override public void run(){
        try {
            sleep(time);
            task.run();
        } catch (Exception e){}
    }
}