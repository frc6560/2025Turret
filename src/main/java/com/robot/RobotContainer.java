package com.robot;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;
import com.robot.Constants.ElevatorConstants;
import com.robot.Constants.OperatorConstants;
import com.robot.commands.BallGrabberCommand;
import com.robot.commands.TurretCommand;
import com.robot.subsystems.BallGrabber;
import com.robot.subsystems.Turret;
import com.robot.subsystems.swervedrive.*;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import com.frc3481.swervelib.SwerveInputStream;



public class RobotContainer {

    // Controllers
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final XboxController firstXbox = new XboxController(0);
    private final XboxController secondXbox = new XboxController(1);
    private final ManualControls controls = new ManualControls(firstXbox, secondXbox);

     // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
  "swerve/falcon"));

    

    // Subsystems
    private final Turret arm = new Turret();
    private final BallGrabber ballGrabber = new BallGrabber();
    //private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/falcon"));

    /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
    drivebase.getSwerveDrive(),
      () -> (Math.pow(driverXbox.getLeftY(), 2)
            * Math.copySign(1, driverXbox.getLeftY())) 
            * -0.9 * ((firstXbox.getLeftTriggerAxis() > 0.25) || (secondXbox.getLeftBumperButton() ) ? 0.6 : 1),
      () -> (Math.pow(driverXbox.getLeftX(), 2)
            * Math.copySign(1, driverXbox.getLeftX())) 
            * -0.9 * ((firstXbox.getLeftTriggerAxis() > 0.25) || (secondXbox.getLeftBumperButton() ) ? 0.6 : 1))
    .withControllerRotationAxis(() -> 
    driverXbox.getRightX() * driverXbox.getRightX() * Math.copySign(1, driverXbox.getRightX()))
    .deadband(OperatorConstants.DEADBAND)
    .scaleTranslation(0.8)
    .allianceRelativeControl(true);; 

/**
* Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
*/
SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                               driverXbox::getRightY)
.headingWhile(true);

/**
* Clone's the angular velocity input stream and converts it to a robotRelative input stream.
*/
SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
.allianceRelativeControl(false);

SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
          () -> -driverXbox.getLeftY(),
          () -> -driverXbox.getLeftX())
      .withControllerRotationAxis(() -> driverXbox.getRawAxis(
          2))
      .deadband(OperatorConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);
// Derive the heading axis with math!
SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                 .withControllerHeadingAxis(() ->
                                                Math.sin(
                                                    driverXbox.getRawAxis(
                                                        2) *
                                                    Math.PI) *
                                                (Math.PI *
                                                 2),
                                            () ->
                                                Math.cos(
                                                    driverXbox.getRawAxis(
                                                        2) *
                                                    Math.PI) *
                                                (Math.PI *
                                                 2))
                 .headingWhile(true)
                 .translationHeadingOffset(true)
                 .translationHeadingOffset(Rotation2d.fromDegrees(
                     0));

/**
* The container for the robot. Contains subsystems, OI devices, and commands.
*/

    

    public RobotContainer() {
      arm.setDefaultCommand(new TurretCommand(arm, controls));

      
      configureBindings();
      // Default elevator command

      DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

      
        
      ballGrabber.setDefaultCommand(new BallGrabberCommand(ballGrabber, controls));
      

    }

    private void configureBindings() {
        // Simulation: Use SmartDashboard buttons for elevator
        // No actual keyboard bindings needed

        Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
        Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngle);
        Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
        Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
            driveDirectAngleKeyboard);
    
        if (RobotBase.isSimulation())
        {
          drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
        } else
        {
          drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        }
    
        if (Robot.isSimulation())
        {
          Pose2d target = new Pose2d(new Translation2d(1, 4),
                                     Rotation2d.fromDegrees(90));
          //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
          driveDirectAngleKeyboard.driveToPose(() -> target,
                                               new ProfiledPIDController(5,
                                                                         0,
                                                                         0,
                                                                         new Constraints(5, 2)),
                                               new ProfiledPIDController(5,
                                                                         0,
                                                                         0,
                                                                         new Constraints(Units.degreesToRadians(360),
                                                                                         Units.degreesToRadians(180))
                                               ));
          driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
          driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
          driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                         () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));
    
    //      driverXbox.b().whileTrue(
    //          drivebase.driveToPose(
    //              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
    //                              );
    
        }
        if (DriverStation.isTest())
        {
          drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!
    
          driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
          driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
          driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
          driverXbox.back().whileTrue(drivebase.centerModulesCommand());
          driverXbox.leftBumper().onTrue(Commands.none());
          driverXbox.rightBumper().onTrue(Commands.none());
          driverXbox.back().onTrue((Commands.runOnce(drivebase::zeroGyro)));
        } else
        {
          driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
          driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
          driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroNoAprilTagsGyro)));
          driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
          driverXbox.rightBumper().onTrue(Commands.none());
        }
    
    }

    public Command getAutonomousCommand() {
      return Commands.run(() -> drivebase.drive(new ChassisSpeeds(-0.5, 0, 0)), drivebase).withTimeout(2);
    }
  
}
