package com.team6560.frc2025;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import com.team6560.frc2025.utility.Enums.*;

import java.util.HashMap;

import com.frc3481.swervelib.math.Matter;
import com.team6560.frc2025.utility.Enums.ReefIndex;
public final class Constants {

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  public static final double MAX_SPEED  = Units.feetToMeters(16.5);

  public static final double WHEEL_COF = 1.19; // subject to much change.

  public static final double ROBOT_WIDTH = 0.965;
  public static final double ROBOT_LENGTH = 0.838;
  // Maximumπspeed of the robot in meters per second, used to limit acceleration.

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds

    // Max velocities and accelerations for auto
    public static final double kMaxAutoVelocity = 3; 
    public static final double kMaxAutoAcceleration = 3; 

    // Max velocities and accelerations for teleop alignment
    public static final double kHandoffVelocity = 0.5;   // formerly 2.1
    public static final double kMaxOmega = Math.toRadians(270);
    public static final double kMaxAlpha = Math.toRadians(360);

    // Tunable constants
    public static final double kS = 0.185; 
    public static final double kV = 1.866; 
    public static final double kA = 0.159; 

    public static final double kP_translation = 3.5;  
    public static final double kI_translation = 0; 
    public static final double kD_translation = 0; 

    public static final double kP_rotation = 1.75;
    public static final double kI_rotation = 0.02;
    public static final double kD_rotation = 0.17;

    public static final double kP_translation_pose = 0.3;  
    public static final double kI_translation_pose = 0; 
    public static final double kD_translation_pose = 0.05; 

    public static final double kP_rotation_pose = 2;
    public static final double kI_rotation_pose = 0.02;
    public static final double kD_rotation_pose = 0.2;

    public static final double kP_translation_intake = 0.85;
    public static final double kI_translation_intake = 0.028;
    public static final double kD_translation_intake = 0.0;


    public static final double kStdvX = 0.08;
    public static final double kStdvY = 0.08;
    public static final double kStdvTheta = 999999;
  }

  public static class OperatorConstants
  {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
    
  }

   public static final class HoodandFlywheelConstants {
        // Hardware IDs
        //public static final int LEFT_FLYWHEEL_MOTOR_ID = 16;
        public static final int RIGHT_FLYWHEEL_MOTOR_ID = 16;
        public static final int HOOD_MOTOR_ID = 46;

        //aneya's canids
        //public static final int RIGHT_FLYWHEEL_MOTOR_ID = 10;
        //public static final int LEFT_FLYWHEEL_MOTOR_ID = 16;
        //public static final int HOOD_MOTOR_ID = 12;
        public static final String CANIVORE_BUS = "";
        
        // Gear Ratios
        public static final double FLYWHEEL_GEAR_RATIO = 25.0 / 24.0;  // 25:24
        public static final double HOOD_GEAR_RATIO = 25.0;  // 25:1 (25 rotations per 1 degree)
        
        // Idle Mode
        public static final double IDLE_RPM = 0.0;
        public static final double HOOD_IDLE_ANGLE = 20.0;
        

        // Test Position
        public static final double FLYWHEEL_TEST_RPM = 100.0;
        public static final double HOOD_TEST_ANGLE = 45.0;
        
        // PID Constants - Flywheel
        public static final double FLYWHEEL_KP = 0.1;
        public static final double FLYWHEEL_KI = 0.0;
        public static final double FLYWHEEL_KD = 0.0;
        public static final double FLYWHEEL_KV = 0.12;
        
        // PID Constants - Hood
        public static final double HOOD_KP = 10.0;
        public static final double HOOD_KI = 0.0;
        public static final double HOOD_KD = 0.5;
        public static final double HOOD_KG = 0.3;  // Gravity compensation
        
        // Tolerances
        public static final double RPM_TOLERANCE = 100.0;
        public static final double HOOD_TOLERANCE = 1.0;
        
        // Soft Limits (degrees)
        public static final double HOOD_MIN_ANGLE = 15.0;
        public static final double HOOD_MAX_ANGLE = 60.0;
        
        // Current Limits (amps)
        public static final double FLYWHEEL_CURRENT_LIMIT = 60.0;
        public static final double HOOD_CURRENT_LIMIT = 40.0;
        
        // Field Positions (meters) - UPDATE THESE FOR YOUR FIELD
        public static final Translation2d BLUE_SIDE = new Translation2d(0.0, 5.5);
        public static final Translation2d RED_SIDE = new Translation2d(16.54, 5.5);
        
        // Lookup Table Data for Linear Regression
        // Key = Distance in meters, Value = RPM or Hood Angle
        // REPLACE THESE WITH YOUR TESTED VALUES
        public static final double[] DISTANCE_TABLE = {
            1.5,   // Very close
            2.0,   // Close
            2.5,   // Medium-close
            3.0,   // Medium
            3.5,   // Medium-far
            4.0,   // Far
            4.5,   // Very far
            5.0    // Maximum range
        };
        
        public static final double[] RPM_TABLE = {
            2500,  // 1.5m
            2700,  // 2.0m
            2900,  // 2.5m
            3100,  // 3.0m
            3300,  // 3.5m
            3500,  // 4.0m
            3700,  // 4.5m
            3900   // 5.0m
        };
        
        public static final double[] HOOD_TABLE = {
            55,  // 1.5m - steep angle
            50,  // 2.0m
            45,  // 2.5m
            40,  // 3.0m
            35,  // 3.5m
            30,  // 4.0m
            25,  // 4.5m
            20   // 5.0m - flat angle
        };
        public static final int HOOD_CANCODER_ID = 44;
    }
  public static final class ElevatorConstants {

    public static int ELEV_LEFT_ID = 14;
    public static int ELEV_RIGHT_ID = 15;
    public static int ELEV_UPPER_LIMIT_SWITCH_ID = 28;
    public static int ELEV_LOWER_LIMIT_SWITCH_ID = 29;

    public static double ELEV_GEAR_RATIO = 2.66;

    public static final double kElevatorTolerance = 1.0;
    public static final class ElevatorStates {

      public static final double STOW = 0.4;
      public static final double L2 = 1.79; 
      public static final double L3 = 7.905;
      public static final double L4 = 19.3;  // prior value: 18.9

      public static final double S_STOW = 0.4;
      public static final double S_L2 = 1.51;
      public static final double S_L3 = 8.15;  
      public static final double S_L4 = 20.8;
    }
  } 

  public static final class WristConstants {

    public static final int M_ID = 16;
    // public static final int SWITCH_ID = 22;  
    public static final int CANCODER_ID = 17; 

    public static final double GEAR_RATIO = 108;
    public static final double kWristTolerance = 1.5;

    public static final double UPPER_SOFT_BOUND = 254.0; 
    public static final double LOWER_SOFT_BOUND = -5.0;

    public static final int LIMIT_SWITCH_PORT_ID = 19;
    public static final class WristStates {
      public static final double STOW = 90.0;
      public static final double PICKUP = 225;  
      public static final double L1 = 200; 
      public static final double L2 = 55; 
      public static final double L4 = 30;  // prior value: 34.8

      public static final double S_STOW = 208.5; 
      public static final double S_L2 = 148.5; 
      public static final double S_L4 = 250; 

      public static final double L1Offset = 0.0;
      public static final double L2Offset = 35.0;
      public static final double L4Offset = 57.0; 
      public static final double StowOffset = 0.0;
      public static final double PickupOffset = 0.0;
    }
  }

  public static final class ClimbConstants {

    public static final int MOTOR_1_ID = 20;
    public static final int MOTOR_2_ID = 21;
    public static final int CANCODER_ID = 22;
    public static final double UPPER_SOFT_BOUND = 0.178; 
    public static final double LOWER_SOFT_BOUND = -0.162;
    public static final double GEAR_RATIO = 1.5; 

  }

  public static final class FieldConstants{
    // AUTO 
    // Left and right are viewed from the DS.
    public static final double BLUE_X = 7.164;

    public static final Pose2d FAR_RIGHT = new Pose2d(10.645, 7.145, Rotation2d.fromDegrees(120));
    public static final Pose2d FAR_LEFT = new Pose2d(10.555, 1.08, Rotation2d.fromDegrees(-120));
    public static final Pose2d RIGHT = new Pose2d(10.645, 5.977, Rotation2d.fromDegrees(120));
    public static final Pose2d LEFT = new Pose2d(10.555, 2.187, Rotation2d.fromDegrees(-120));
    public static final Pose2d CENTER_RED = new Pose2d(10.413, 4.0, Rotation2d.fromDegrees(180));

    public static final Pose2d FAR_LEFT_BLUE = new Pose2d(BLUE_X, 6.92, Rotation2d.fromDegrees(0));
    public static final Pose2d FAR_RIGHT_BLUE = new Pose2d(BLUE_X, 1, Rotation2d.fromDegrees(0));
    public static final Pose2d LEFT_BLUE = new Pose2d(7.039, 5.897, Rotation2d.fromDegrees(60));
    public static final Pose2d RIGHT_BLUE = new Pose2d(6.908, 2.110, Rotation2d.fromDegrees(-60));
    public static final Pose2d CENTER_BLUE = new Pose2d(BLUE_X, 4.0, Rotation2d.fromDegrees(0));

    public static final double Elevator_top_algae = 0.0;
    public static final double Wrist_top_algae = 0.0;
    public static final double Elevator_low_algae = 0.0;
    public static final double Wrist_low_algae = 0.0;

    public static Pose2d getFarRight(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? FAR_RIGHT : FAR_RIGHT_BLUE;
    }

    public static Pose2d getFarLeft(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? FAR_LEFT : FAR_LEFT_BLUE;
    }

    public static Pose2d getRight(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? RIGHT : RIGHT_BLUE;
    }

    public static Pose2d getLeft(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? LEFT : LEFT_BLUE;
    }

    public static Pose2d getCenter(DriverStation.Alliance alliance){
      return alliance == DriverStation.Alliance.Red ? CENTER_RED : CENTER_BLUE;
    }
  }
}