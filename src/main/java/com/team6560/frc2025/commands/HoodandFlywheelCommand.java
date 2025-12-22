// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025.commands;

import com.team6560.frc2025.subsystems.HoodandFlywheel;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6560.frc2025.Constants.HoodandFlywheelConstants;
import edu.wpi.first.wpilibj2.command.Command;
import com.team6560.frc2025.controls.XboxControls;

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
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // A button - Move hood to 20 degrees (one-time action)
    boolean hoodButton = controls.moveHoodTo20();
    if (hoodButton && !lastHoodButton) {  
      hoodAtTarget = true;
      //setHoodAngle(); 
      hoodandflywheel.setHoodAngle(45); 
            
            
          }
          lastHoodButton = hoodButton;
      
          // Y button - Spin flywheel to 500 RPM (one-time action)
          boolean flywheelButton = controls.spinFlywheelTo500();
          if (flywheelButton && !lastFlywheelButton) {  // Rising edge detection
            flywheelSpinning = true;
            currentRPM = 500.0;
            hoodandflywheel.setFlywheelRPM(currentRPM);
            System.out.println("Flywheel spinning at 500 RPM");
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
            boolean decreaseButton = controls.decreaseRPM();
            if (decreaseButton && !lastDecreaseButton) {  // Rising edge detection
              currentRPM -= 50;
              currentRPM = Math.max(currentRPM, 0);  // Don't go below 0
              hoodandflywheel.setFlywheelRPM(currentRPM);
              System.out.println("Decreased RPM to: " + currentRPM);
            }
            lastDecreaseButton = decreaseButton;
          }
      
          // Zero hood if needed (Start button)
          if (controls.zeroHood()) {
            hoodandflywheel.zeroHood();
            System.out.println("Hood encoder zeroed");
          }
        }
        private void setHoodAngle() {
          // TODO Auto-generated method stub
          throw new UnsupportedOperationException("Unimplemented method 'setHoodAngle'");
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
