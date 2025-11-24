package com.robot.commands;

import com.robot.ManualControls;
import com.robot.Constants.TurretConstants;
import com.robot.subsystems.Turret;
import com.robot.subsystems.Turret.State;

import com.robot.utility.LimelightHelpers;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.robot.subsystems.swervedrive.SwerveSubsystem;



public class TurretCommand extends Command {
    private final Turret turret;
    private final ManualControls controls;
    public State targetState;

    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Turret");
    private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
    private final NetworkTableEntry ntPosition = ntTable.getEntry("Turret position");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");
    
    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d root = mech.getRoot("TurretRoot", 30, 5);
    private final MechanismLigament2d turretLigament =
        root.append(new MechanismLigament2d("Turret", 20, 90));

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
        double angle = turret.getTurretAngleDeg();
        
        // Update NetworkTables
        ntAngle.setDouble(angle);
        ntPosition.setDouble(angle);
        
        // Update Mechanism2d visualization
        turretLigament.setAngle(angle);

        // Update SmartDashboard
        SmartDashboard.putNumber("Current Angle", angle);

        if (turret.getTurretAngleDeg() < 0) {
            turret.setGoal(((turret.getTurretAngleDeg() % 360) + 360) % 360);
        } else if (turret.getTurretAngleDeg() > 360) {
            turret.setGoal(((turret.getTurretAngleDeg() % 360) + 360) % 360);
        }
        
        // if (LimelightHelpers.getTV("Limelight1")) {
        //     LimelightHelpers.getTX("Limelight1");
        //     double targetAngle = LimelightHelpers.getTX("Limelight1") + turret.getTurretAngleDeg();
        //     targetAngle = ((targetAngle % 360) + 360) % 360;
        //     turret.setGoal(targetAngle);
        // } else {
        
        
            Pose2d robotPose = turret.getDrivebase().getPose();
            Pose2d fieldTarget = turret.getFieldTarget();

            double dx = fieldTarget.getX() - robotPose.getX();
            double dy = fieldTarget.getY() - robotPose.getY();
            double odomTargetAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
        
            double robotHeadingDeg = robotPose.getRotation().getDegrees();
            double turretBaseDeg = odomTargetAngleDeg - robotHeadingDeg;

            turretBaseDeg = ((turretBaseDeg % 360) + 360) % 360;
            turret.setGoal(turretBaseDeg);
        //}

        

        
            double targetDeg = turret.getGoalValue();
            double currentDeg = turret.getTurretAngleDeg();

            
    }
    
    public void periodic() {
        
        //}
        

    }
    
    @Override
    public boolean isFinished() {
        return false; 
    }

    @Override
    public void end(boolean interrupted) {
    }
    
}