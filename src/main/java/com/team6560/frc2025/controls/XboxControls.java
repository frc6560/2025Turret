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

// Hood and Flywheel Controls
// A button - Move hood to 20 degrees
public boolean moveHoodTo20() {
    return firstXbox.getAButton();  
}

// Y button - Spin flywheel to 500 RPM
public boolean spinFlywheelTo500() {
    return firstXbox.getYButton();  
}

// X button - Decrease RPM by 50
public boolean decreaseRPM() {
    return firstXbox.getXButton();
}

// B button - Increase RPM by 50
public boolean increaseRPM() {
    return firstXbox.getBButton(); 
}

// Old functions kept for compatibility (not used in your new setup)
public boolean aimhoodandflywheel() {
    return firstXbox.getRightTriggerAxis() > 0.25;   
}

public boolean manualhoodandflywheel() {
  return firstXbox.getLeftTriggerAxis() > 0.25;  
}

public boolean stophoodandflywheel() {
    return false;  // Not used
}

public boolean idlehoodandflywheel() {
    return false;  // Not used
}

public boolean zeroHood() {
    return firstXbox.getStartButton();  // Moved to Start button
}

public boolean increaseHood() {
  return firstXbox.getLeftBumperButton();
}

public boolean decreaseHood() {
  return firstXbox.getRightBumperButton();
}

public boolean moveHoodToTestPosition() {
  return firstXbox.getBackButton();  // Not used in current setup
}
    
    
}