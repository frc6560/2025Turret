package com.team6560.frc2025.controls;

import edu.wpi.first.wpilibj.XboxController;

public class XboxControls {
    private final XboxController secondXbox;
    private final XboxController firstXbox;
    
    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }

      private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.01);
    
        // Square the axis
        value = Math.copySign(value * value, value);
    
        return value;
      }
      
    public XboxControls(XboxController firstXbox, XboxController secondXbox) {
        this.secondXbox = secondXbox;
        this.firstXbox = firstXbox;
        
    }

    // --- CLIMB ---

    public boolean getClimbDown() {
      return firstXbox.getBackButton(); 
    }

    public boolean getClimbUp() {
     return firstXbox.getBackButton();
    }

    // // --- SUPERSTRUCTURE ---
    public boolean goToPickup(){
       return firstXbox.getBackButton();
    }

    // --- END EFFECTORS ---

   public boolean runGrabberOuttake() {
     return firstXbox.getLeftTriggerAxis() > 0.25;
    }

public boolean aimhoodandflywheel() {
    return firstXbox.getRightTriggerAxis() > 0.25;   
}

public boolean manualhoodandflywheel() {
  return firstXbox.getLeftTriggerAxis() > 0.25;  
}

public boolean stophoodandflywheel() {
    return firstXbox.getAButton();  
}

public boolean idlehoodandflywheel() {
    return firstXbox.getBButton(); 
}

public boolean zeroHood() {
    return firstXbox.getYButton();  
}

// Manual adjustment controls
public boolean increaseRPM() {
  return firstXbox.getYButton();
}

public boolean decreaseRPM() {
  return firstXbox.getStartButton();
}

public boolean increaseHood() {
  return firstXbox.getLeftBumperButton();
}

public boolean decreaseHood() {
  return firstXbox.getRightBumperButton();
}

public boolean moveHoodToTestPosition() {
  // TODO Auto-generated method stub
  return firstXbox.getXButton();
}
    
    
}