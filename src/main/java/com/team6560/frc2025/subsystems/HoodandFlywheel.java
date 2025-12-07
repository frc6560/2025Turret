// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6560.frc2025.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.team6560.frc2025.Constants.HoodandFlywheelConstants;

public class HoodandFlywheel extends SubsystemBase {
  public interface getSwerveDrive {
    public Pose2d getPose();
    public Translation2d getVelocity();
  }

  //Linear regression coefficients 
  private static class RegressionCoefficients{
    public final double slope; 
    public final double intercept;
    public final double rSquared; 

    public RegressionCoefficients(double slope, double intercept, double rSquared){
      this.slope = slope;
      this.intercept = intercept;
      this.rSquared = rSquared;
    }
  }

  //hardware 
  private final TalonFX leftFlywheelMotor; 
  private final TalonFX rightFlywheelMotor;
  private final TalonFX hoodMotor;

  // Motor controls 
  private final VelocityVoltage flywheelVelocityControl;
  private final PositionVoltage hoodPositionControl;

  //Drivetrain reference for pose 
  private final getSwerveDrive drivetrain;

  // linear regression models 
  private final RegressionCoefficients rpmRegression;
  private final RegressionCoefficients hoodRegression;

  // current targets 
  private double targetRPM = HoodandFlywheelConstants.IDLE_RPM; 
  private double targetHoodAngle = HoodandFlywheelConstants.HOOD_IDLE_ANGLE;

  //Netwrork table 
  private final NetworkTable hoodandflywheelTable;
  /** Creates a new HoodandFlywheel. */
  public HoodandFlywheel(getSwerveDrive drivetrain) {
    this.drivetrain = drivetrain;
    this.hoodandflywheelTable = NetworkTableInstance.getDefault().getTable("HoodandFlywheel");

    // Initialize hardware
    this.leftFlywheelMotor = new TalonFX(HoodandFlywheelConstants.LEFT_FLYWHEEL_MOTOR_ID);
    this.rightFlywheelMotor = new TalonFX(HoodandFlywheelConstants.RIGHT_FLYWHEEL_MOTOR_ID);
    this.hoodMotor = new TalonFX(HoodandFlywheelConstants.HOOD_MOTOR_ID);

    // config harware 
    configureFlywheel(leftFlywheelMotor, false); 
    configureFlywheel(rightFlywheelMotor, true);
    configureHood(hoodMotor);

    // Initialize motor controls
    flywheelVelocityControl = new VelocityVoltage(0).withSlot(0);
    hoodPositionControl = new PositionVoltage(0).withSlot(0);
    
    // Calculate linear regression models from lookup tables
    rpmRegression = calculateLinearRegression(
        HoodandFlywheelConstants.DISTANCE_TABLE,
        HoodandFlywheelConstants.RPM_TABLE
    );
    hoodRegression = calculateLinearRegression(
        HoodandFlywheelConstants.DISTANCE_TABLE,
        HoodandFlywheelConstants.HOOD_TABLE
    );
    // Print regression results
    System.out.println("=== Shooter Linear Regression ===");
    System.out.printf("RPM: y = %.2fx + %.2f (R² = %.3f)%n", 
        rpmRegression.slope, rpmRegression.intercept, rpmRegression.rSquared);
    System.out.printf("Hood: y = %.2fx + %.2f (R² = %.3f)%n",
        hoodRegression.slope, hoodRegression.intercept, hoodRegression.rSquared);
    
    // Start at idle
    setIdle();
}

/**
 * Calculates linear regression coefficients using least squares method
 * Formula: y = mx + b
 * 
 * @param x Distance values (independent variable)
 * @param y RPM or hood angle values (dependent variable)
 * @return Regression coefficients (slope, intercept, R²)
 */
private RegressionCoefficients calculateLinearRegression(double[] x, double[] y) {
    int n = x.length;
    
    // Calculate means
    double sumX = 0, sumY = 0;
    for (int i = 0; i < n; i++) {
        sumX += x[i];
        sumY += y[i];
    }
    double meanX = sumX / n;
    double meanY = sumY / n;
    
    // Calculate slope (m) using least squares
    // m = Σ[(xi - x̄)(yi - ȳ)] / Σ[(xi - x̄)²]
    double numerator = 0;
    double denominator = 0;
    for (int i = 0; i < n; i++) {
        numerator += (x[i] - meanX) * (y[i] - meanY);
        denominator += (x[i] - meanX) * (x[i] - meanX);
    }
    double slope = numerator / denominator;
    
    // Calculate intercept (b)
    // b = ȳ - m * x̄
    double intercept = meanY - slope * meanX;
    
    // Calculate R² (coefficient of determination)
    // R² = 1 - (SSresidual / SStotal)
    double ssTotal = 0;
    double ssResidual = 0;
    for (int i = 0; i < n; i++) {
        double predicted = slope * x[i] + intercept;
        ssTotal += (y[i] - meanY) * (y[i] - meanY);
        ssResidual += (y[i] - predicted) * (y[i] - predicted);
    }
    double rSquared = 1 - (ssResidual / ssTotal);
    
    return new RegressionCoefficients(slope, intercept, rSquared);
}

/**
 * Predicts value using linear regression: y = mx + b
 */
private double predictValue(RegressionCoefficients regression, double x) {
    return regression.slope * x + regression.intercept;
}

private void configureFlywheel(TalonFX motor, boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // PID configuration
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = HoodandFlywheelConstants.FLYWHEEL_KP;
    slot0.kI = HoodandFlywheelConstants.FLYWHEEL_KI;
    slot0.kD = HoodandFlywheelConstants.FLYWHEEL_KD;
    slot0.kV = HoodandFlywheelConstants.FLYWHEEL_KV;
    config.Slot0 = slot0;
    
    // Motor configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.SupplyCurrentLimit = HoodandFlywheelConstants.FLYWHEEL_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    if (inverted) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }
    
    motor.getConfigurator().apply(config);
}

private void configureHood(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    
    // PID configuration
    Slot0Configs slot0 = new Slot0Configs();
    slot0.kP = HoodandFlywheelConstants.HOOD_KP;
    slot0.kI = HoodandFlywheelConstants.HOOD_KI;
    slot0.kD = HoodandFlywheelConstants.HOOD_KD;
    slot0.kG = HoodandFlywheelConstants.HOOD_KG;
    config.Slot0 = slot0;
    
    // Motor configuration
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.SupplyCurrentLimit = HoodandFlywheelConstants.HOOD_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    
    // Soft limits
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 
        HoodandFlywheelConstants.HOOD_MAX_ANGLE * HoodandFlywheelConstants.HOOD_GEAR_RATIO;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 
        HoodandFlywheelConstants.HOOD_MIN_ANGLE * HoodandFlywheelConstants.HOOD_GEAR_RATIO;
    
        motor.getConfigurator().apply(config);
    }
         /**
         * Aims flywheel and hood using global pose and linear regression
         * Gets robot position, calculates distance to speaker, uses regression to predict values
         */
        public void aimhoodandflywheel() {
      // Get robot position from drivetrain
      Pose2d robotPose = drivetrain.getPose();
      
      // Get target position based on alliance
      Translation2d target = getTargetPosition();
      
      // Calculate distance to target using global pose
      double distance = robotPose.getTranslation().getDistance(target);
      
      // Use linear regression to predict RPM and hood angle
      targetRPM = predictValue(rpmRegression, distance);
      targetHoodAngle = predictValue(hoodRegression, distance);
      
      // Clamp hood angle to limits
      targetHoodAngle = Math.max(HoodandFlywheelConstants.HOOD_MIN_ANGLE, 
                                   Math.min(HoodandFlywheelConstants.HOOD_MAX_ANGLE, targetHoodAngle));
      
      // Set motors
      setFlywheelRPM(targetRPM);
      setHoodAngle(targetHoodAngle);
      
      // Telemetry
      SmartDashboard.putNumber("Shooter/Distance", distance);
      SmartDashboard.putNumber("Shooter/Target RPM", targetRPM);
      SmartDashboard.putNumber("Shooter/Target Hood", targetHoodAngle);
  }
  
  /**
   * Gets target speaker position based on alliance color
   */
  private Translation2d getTargetPosition() {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent() && alliance.get() == Alliance.Red) {
          return HoodandFlywheelConstants.RED_SIDE;
      }
      return HoodandFlywheelConstants.BLUE_SIDE;
  }
  
  /**
   * Sets flywheel RPM (accounts for 25:24 gear ratio)
   */
  public void setFlywheelRPM(double rpm) {
      // Convert RPM through gear ratio
      double motorRPM = rpm * HoodandFlywheelConstants.FLYWHEEL_GEAR_RATIO;
      
      // Convert to rotations per second for Phoenix 6
      double rps = motorRPM / 60.0;
      
      leftFlywheelMotor.setControl(flywheelVelocityControl.withVelocity(rps));
      rightFlywheelMotor.setControl(flywheelVelocityControl.withVelocity(rps));
  }
  
  /**
   * Sets hood angle in degrees (accounts for 25:1 gear ratio)
   */
  public void setHoodAngle(double degrees) {
      // Clamp to limits
      degrees = Math.max(HoodandFlywheelConstants.HOOD_MIN_ANGLE, 
                        Math.min(HoodandFlywheelConstants.HOOD_MAX_ANGLE, degrees));
      
      // Convert to motor rotations (25:1 gear ratio)
      double motorRotations = degrees * HoodandFlywheelConstants.HOOD_GEAR_RATIO;
      
      hoodMotor.setControl(hoodPositionControl.withPosition(motorRotations));
  }
  
  /**
   * Sets shooter to idle mode (1000 RPM, 20 degrees)
   */
  public void setIdle() {
      targetRPM = HoodandFlywheelConstants.IDLE_RPM;
      targetHoodAngle = HoodandFlywheelConstants.HOOD_IDLE_ANGLE;
      setFlywheelRPM(targetRPM);
      setHoodAngle(targetHoodAngle);
  }
  
  /**
   * Stops all motors
   */
  public void stopMotors() {
      leftFlywheelMotor.stopMotor();
      rightFlywheelMotor.stopMotor();
      hoodMotor.stopMotor();
  }
  
  /**
   * Checks if flywheel and hood are at target values within tolerance
   */
  public boolean isReadyToShoot() {
      double rpmError = Math.abs(getFlywheelRPM() - targetRPM);
      double hoodError = Math.abs(getHoodAngle() - targetHoodAngle);
      
      return rpmError < HoodandFlywheelConstants.RPM_TOLERANCE && 
             hoodError < HoodandFlywheelConstants.HOOD_TOLERANCE;
  }
  
  /**
   * Gets current flywheel RPM (accounts for gear ratio)
   */
  public double getFlywheelRPM() {
      double rps = leftFlywheelMotor.getVelocity().getValueAsDouble();
      double motorRPM = rps * 60.0;
      return motorRPM / HoodandFlywheelConstants.FLYWHEEL_GEAR_RATIO;
  }
  
  /**
   * Gets current hood angle in degrees (accounts for gear ratio)
   */
  public double getHoodAngle() {
      double motorRotations = hoodMotor.getPosition().getValueAsDouble();
      return motorRotations / HoodandFlywheelConstants.HOOD_GEAR_RATIO;
  }
  
  /**
   * Zeros the hood encoder to current position
   */
  public void zeroHood() {
      hoodMotor.setPosition(0);
  }
  
  /**
   * Gets R² quality metric for RPM regression (0-1, higher is better)
   */
  public double getRPMRegressionQuality() {
      return rpmRegression.rSquared;
  }
  
  /**
   * Gets R² quality metric for hood regression (0-1, higher is better)
   */
  public double getHoodRegressionQuality() {
      return hoodRegression.rSquared;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
