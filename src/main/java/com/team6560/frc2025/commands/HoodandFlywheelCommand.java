// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.HoodandFlywheel;

import com.team6560.frc2025.Constants.HoodandFlywheelConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2025.controls.XboxControls;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodandFlywheelCommand extends Command {

  public enum State {
    IDLE,      // Spinning at idle RPM (1000)
    AIMING,    // Aiming at target using global pose + linear regression
    MANUAL,    // Manual control for testing
    STOPPED,    // Full stop
    TEST_POSITION, // Move to test position
}

private final HoodandFlywheel hoodandflywheel;
private final XboxControls controls;
private State state = State.IDLE;

// Manual test values
private double manualRPM = 2500.0;
private double manualHood = 45.0;

  /** Creates a new HoodandFlywheelCommand. */
  public HoodandFlywheelCommand(HoodandFlywheel hoodandflywheel, XboxControls controls) {
    this.hoodandflywheel = hoodandflywheel;
    this.controls = controls;
    addRequirements(hoodandflywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = State.IDLE;
    hoodandflywheel.setIdle(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (controls.moveHoodToTestPosition()) {
      state = State.TEST_POSITION;
     // State transitions based on operator input from XboxControls
    } else if (controls.aimhoodandflywheel()) {
      // Aim using global pose and linear regression
      state = State.AIMING;
  } else if (controls.manualhoodandflywheel()) {
      // Manual mode for testing
      state = State.MANUAL;
  } else if (controls.stophoodandflywheel()) {
      // Stop everything
      state = State.STOPPED;
  } else if (controls.idlehoodandflywheel()) {
      // Return to idle
      state = State.IDLE;

  }
  
  // Execute based on current state
  switch (state) {
      case IDLE:
          // Spin at idle RPM (1000 RPM)
          hoodandflywheel.setIdle();
          break;
        
        case TEST_POSITION:
            // Move to test position
            hoodandflywheel.setHoodAngle(HoodandFlywheelConstants.HOOD_TEST_ANGLE);
            hoodandflywheel.setFlywheelRPM(HoodandFlywheelConstants.FLYWHEEL_TEST_RPM);
            break;


    
          
      case AIMING:
          // Use global pose and linear regression to aim
          hoodandflywheel.aimhoodandflywheel();
          break;
          
      case MANUAL:
          // Manual adjustments
          if (controls.increaseRPM()) {
              manualRPM += 50;
          } else if (controls.decreaseRPM()) {
              manualRPM -= 50;
          }
          
          if (controls.increaseHood()) {
              manualHood += 1;
          } else if (controls.decreaseHood()) {
              manualHood -= 1;
          }
          
          // Clamp values to valid ranges
          manualRPM = Math.max(0, Math.min(6000, manualRPM));
          manualHood = Math.max(HoodandFlywheelConstants.HOOD_MIN_ANGLE, 
                                 Math.min(HoodandFlywheelConstants.HOOD_MAX_ANGLE, manualHood));
          
          hoodandflywheel.setFlywheelRPM(manualRPM);
          hoodandflywheel.setHoodAngle(manualHood);
          break;
          
      case STOPPED:
          hoodandflywheel.stopMotors();
          break;
  }
  
  // Zero hood (anytime)
  if (controls.zeroHood()) {
      hoodandflywheel.zeroHood();
  }
}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodandflywheel.setIdle(); 
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  
}
}
