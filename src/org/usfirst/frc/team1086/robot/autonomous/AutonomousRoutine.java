package org.usfirst.frc.team1086.robot.autonomous;

public abstract class AutonomousRoutine extends AutonomousCommander {
    public AutonomousRoutine(){
        super();
    }
    public abstract void init();
    public void begin(){
    	init();
        start();
    }
}