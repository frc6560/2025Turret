package com.robot.commands;

import com.robot.ManualControls;
import com.robot.Constants.TurretConstants;
import com.robot.subsystems.Turret;
import com.robot.subsystems.Turret.State;

import edu.wpi.first.wpilibj2.command.Command;

public class TurretCommand extends Command {
    private final Turret turret;
    private final ManualControls controls;
    public State targetState;
    
    public TurretCommand(Turret turret, ManualControls controls) {
        this.turret = turret;
        this.controls = controls;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){

    }

    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
        turret.stopMotor();
    }
}