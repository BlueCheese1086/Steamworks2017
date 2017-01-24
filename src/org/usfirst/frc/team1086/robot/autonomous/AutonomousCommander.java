package org.usfirst.frc.team1086.robot.autonomous;

import java.util.HashMap;

public class AutonomousCommander {
    HashMap<Integer, Double> sectionTimes = new HashMap();
    HashMap<Integer, Runnable> sectionActions = new HashMap();
    HashMap<Integer, Runnable> sectionStartActions = new HashMap();
    int section;
    double endTime;
    boolean started = false;
    Thread manager = new Thread(){
        @Override public void run(){
            tick();
            try {
                sleep(10);
                if(started)
                    run();
            } catch (Exception e) {}
        }
    };
    public AutonomousCommander(){}
    public void addSection(double time, Runnable ru){
        addSection(time, ru, () -> {});
    }
    public void addSection(Action a){
        addSection(Double.POSITIVE_INFINITY, () -> {
            if(a.run())
                next();
        }, () -> {});
    }
    public void addSection(double time, Runnable ru, Runnable start){
        sectionTimes.put(sectionTimes.size(), time);
        sectionActions.put(sectionActions.size(), ru);
        sectionStartActions.put(sectionStartActions.size(), start);
    }
    public void addSection(Action a, Runnable ru){
        addSection(Double.POSITIVE_INFINITY, () -> {
            if(a.run())
                next();
        }, ru);
    }
    public void tick(){
        if(System.currentTimeMillis() >= endTime)
            next();
        sectionActions.get(section).run();
    }
    public void start(){
        if(sectionTimes.values().stream().mapToDouble(s -> s).sum() > 15000)
            System.out.println("WARNING: Autonomous duration exceeds 15 seconds");
        sectionTimes.put(sectionTimes.size(), Double.POSITIVE_INFINITY);
        sectionActions.put(sectionActions.size(), () -> stop());
        sectionStartActions.put(sectionActions.size(), () -> stop());
        goToSection(0);
        started = true;
        manager.start();
    }
    public void stop(){
        started = false;
        sectionTimes.remove(sectionTimes.size() - 1);
        sectionActions.remove(sectionActions.size() - 1);
        sectionStartActions.remove(sectionStartActions.size() - 1);
    }
    public void goToSection(int n){
        section = n;
        endTime = System.currentTimeMillis() + sectionTimes.get(n);
        sectionStartActions.get(n).run();
    }
    public void next(){
        goToSection(section + 1);
    }
    public void restartSection(){
        goToSection(section);
    }
}