// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package com.robot.commands;

import com.robot.ManualControls;
import com.robot.subsystems.BallGrabber;

import edu.wpi.first.wpilibj2.command.Command;

 public class BallGrabberCommand extends Command {
    
    final BallGrabber ballGrabber;
    final ManualControls controls;

    public BallGrabberCommand(BallGrabber grabber, ManualControls controls) {
        this.ballGrabber = grabber;
        this.controls = controls;
        addRequirements(grabber);

    }
    @Override
    public void initialize(){

        ballGrabber.stop();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
 }