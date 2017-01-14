package org.usfirst.frc.team1086.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

public class Robot extends IterativeRobot {
    final String defaultAuto = "Default";
    final String customAuto = "My Auto";
    String autoSelected;
    SendableChooser<String> chooser = new SendableChooser<>();

    @Override
    public void robotInit(){}

    @Override
    public void autonomousInit(){}

    @Override
    public void autonomousPeriodic(){}

    @Override
    public void teleopPeriodic(){}

    @Override
    public void testPeriodic(){}
}

