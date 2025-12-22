// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.HoodandFlywheel;

import com.team6560.frc2025.Constants.HoodandFlywheelConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2025.controls.XboxControls;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HoodandFlywheelCommand extends Command {

private final HoodandFlywheel hoodandflywheel;
private final XboxControls controls;

// Track current state
private boolean hoodAtTarget = false;  // Hood at 20 degrees
private boolean flywheelSpinning = false;  // Flywheel spinning
private double currentRPM = 0.0;  // Current flywheel RPM

// Button press tracking (for single press detection)
private boolean lastIncreaseButton = false;
private boolean lastDecreaseButton = false;
private boolean lastHoodButton = false;
private boolean lastFlywheelButton = false;

  /** Creates a new HoodandFlywheelCommand. */
  public HoodandFlywheelCommand(HoodandFlywheel hoodandflywheel, XboxControls controls) {
    this.hoodandflywheel = hoodandflywheel;
    this.controls = controls;
    addRequirements(hoodandflywheel);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Start with everything off
    hoodAtTarget = false;
    flywheelSpinning = false;
    currentRPM = 0.0;
    hoodandflywheel.stopMotors();
    
    // Put default target RPM on Shuffleboard so you can edit it
    SmartDashboard.putNumber("Flywheel Target RPM", 500.0);
    
    System.out.println("HoodandFlywheel initialized - all motors stopped");
    System.out.println("Use Shuffleboard to set 'Flywheel Target RPM' and press Y button to spin");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // A button - Move hood to 20 degrees (one-time action)
    boolean hoodButton = controls.moveHoodTo20();
    if (hoodButton && !lastHoodButton) {  // Rising edge detection
      hoodAtTarget = true;
      hoodandflywheel.setHoodAngle(20.0);
      System.out.println("Hood moving to 20 degrees");
    }
    lastHoodButton = hoodButton;

    // Y button - Spin flywheel to RPM from Shuffleboard
    boolean flywheelButton = controls.spinFlywheelTo500();
    if (flywheelButton && !lastFlywheelButton) {  // Rising edge detection
      flywheelSpinning = true;
      // Read target RPM from Shuffleboard instead of hardcoded 500
      currentRPM = SmartDashboard.getNumber("Flywheel Target RPM", 500.0);
      // Safety: Clamp to 0-6000 RPM to prevent damage
      currentRPM = Math.max(0, Math.min(6000, currentRPM));
      hoodandflywheel.setFlywheelRPM(currentRPM);
      System.out.println("Flywheel spinning at " + currentRPM + " RPM (from Shuffleboard)");
    }
    lastFlywheelButton = flywheelButton;

    // Only allow RPM adjustments if flywheel is spinning
    if (flywheelSpinning) {
      // B button - Increase RPM by 50
      boolean increaseButton = controls.increaseRPM();
      if (increaseButton && !lastIncreaseButton) {  // Rising edge detection
        currentRPM += 50;
        currentRPM = Math.min(currentRPM, 6000);  // Cap at 6000 RPM
        hoodandflywheel.setFlywheelRPM(currentRPM);
        System.out.println("Increased RPM to: " + currentRPM);
      }
      lastIncreaseButton = increaseButton;

      // X button - Decrease RPM by 50
      //update: set rpm to 3000
      boolean decreaseButton = controls.setRpm3000();
      if (decreaseButton && !lastDecreaseButton) {  // Rising edge detection
        currentRPM = 3000;
        currentRPM = Math.max(currentRPM, 0);  // Don't go below 0
        hoodandflywheel.setFlywheelRPM(currentRPM);
        System.out.println("Decreased RPM to: " + currentRPM);
      }
      lastDecreaseButton = decreaseButton;
    }

    // Update Shuffleboard with current RPM while running
    if (flywheelSpinning) {
      SmartDashboard.putNumber("Flywheel Current RPM", hoodandflywheel.getFlywheelRPM());
    }

    // Zero hood if needed (Start button)
    if (controls.zeroHood()) {
      hoodandflywheel.zeroHood();
      System.out.println("Hood encoder zeroed");
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hoodandflywheel.stopMotors();
    System.out.println("HoodandFlywheel command ended - motors stopped");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  
}
}
