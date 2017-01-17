package org.usfirst.frc.team1086.robot.autonomous;

public abstract class AutonomousRoutine {
    public AutonomousCommander ac;
    public AutonomousRoutine(){
        ac = new AutonomousCommander();
    }
    public abstract void init();
    public void begin(){
        ac.start();
        new ASyncDelayedTask(15000, () -> {
            ac.stop();
        }).start();
    }
}