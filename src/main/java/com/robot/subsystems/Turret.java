package com.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.robot.Constants.ElevatorConstants;
import com.robot.Constants.TurretConstants;

import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;


public class Turret extends SubsystemBase {

/*
    private final DigitalInput topLimitSwitch = new DigitalInput(ElevatorConstants.TopLimitSwitchID);
    private final DigitalInput botLimitSwitch = new DigitalInput(ElevatorConstants.BotLimitSwitchID);
*/

    private final TalonFX turretMotor = new TalonFX(TurretConstants.MOTOR_ID, "Canivore");
    
    private final ArmFeedforward turretFeedForward = new ArmFeedforward(
        TurretConstants.kS, TurretConstants.kV, TurretConstants.kA);
    private double targetPos = 0;

    public final TrapezoidProfile.Constraints turretConstraints =
    new TrapezoidProfile.Constraints(TurretConstants.kMaxV, TurretConstants.kMaxA);

    public final TrapezoidProfile turretTrapezoidProfile = new TrapezoidProfile(turretConstraints);

    public TrapezoidProfile.State turGoalState = new TrapezoidProfile.State();
    public TrapezoidProfile.State turSetpointState = new TrapezoidProfile.State();

    private final PIDController pidController = new PIDController(
        TurretConstants.kP,
        TurretConstants.kI,
        TurretConstants.kD
    );

    // NetworkTables
    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Turret");
    private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
    private final NetworkTableEntry ntPosition = ntTable.getEntry("Turret position");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");

    // Mechanism2d visualization
    private final Mechanism2d mech = new Mechanism2d(60, 60);
    private final MechanismRoot2d root = mech.getRoot("TurretRoot", 30, 5);
    private final MechanismLigament2d turretLigament =
        root.append(new MechanismLigament2d("Turret", 20, 90));

    public enum State {
        STOW,
        PICKUP,
        REEF_LOW,
        REEF_HIGH,
        BARGE,
        PROCESSOR,
        IN_MOTION
    }

    public Turret() {

        // Configure TalonFX
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // PID and feedforward configuration
        Slot0Configs slot0 = talonFXConfigs.Slot0;
        slot0.kS = TurretConstants.kS;
        slot0.kV = TurretConstants.kV;
        slot0.kA = TurretConstants.kA;
        slot0.kP = TurretConstants.kP;
        slot0.kI = TurretConstants.kI;
        slot0.kD = TurretConstants.kD;

        // Set motor to brake mode
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        turretMotor.getConfigurator().apply(talonFXConfigs.withSlot0(slot0));
        // Initialize Mechanism2d visualization
        turretLigament.setLength(TurretConstants.ARM_LENGTH_METERS * 50);
        turretLigament.setAngle(TurretConstants.STOW_POSITION_DEG);
        SmartDashboard.putData("TurretMech2d", mech);
    }

    public void setSetpoint(TrapezoidProfile.State nextSetpoint) { turSetpointState = nextSetpoint; }
    
    public TrapezoidProfile.State getSetpoint() { return turSetpointState; }

    /**
     * Set the target angle for the turret using Motion Magic control
     * @param goalDeg Target angle in degrees
     */
    public void setGoal(double goalDeg) {
        turGoalState = new TrapezoidProfile.State(goalDeg/360, 0); 
        setControl();
        ntTargetPos.setDouble(goalDeg);
    }

    public TrapezoidProfile.State getGoal() { return turGoalState; }

    
    public double getTurretAngleDeg() {
        // Get position from TalonFX internal encoder (in rotations)
        double rotations = turretMotor.getPosition().getValueAsDouble();
        // Convert rotations back to degrees (reverse the 81:1 gear ratio)
        double angle = rotations * 360;
        return angle;
    }

    /**
     * Reset the internal encoder to zero at the current position
     * Call this when the turret is in a known position (e.g., stow position)
     */
    public void resetEncoder() {
        turretMotor.setPosition(0);
    }

    /**
     * Reset encoder and set it to a specific angle
     * @param currentAngleDeg The actual current angle of the turret in degrees
     */
    public void resetEncoderToAngle(double currentAngleDeg) {
        double rotations = (currentAngleDeg / 360.0);
        turretMotor.setPosition(rotations);
    }

    /**
     * Get the current state of the turret based on its position
     * @return Current turret state
     */
    public State getState() {
        double padding = 1.5; // Tolerance in degrees
        double angle = getTurretAngleDeg();

        if (Math.abs(angle - TurretConstants.STOW_POSITION_DEG) < padding) {
            return State.STOW;
        } else if (Math.abs(angle - TurretConstants.PICKUP_POSITION_DEG) < padding) {
            return State.PICKUP;
        } else if (Math.abs(angle - TurretConstants.REEF_POSITION_DEG_low) < padding) {
            return State.REEF_LOW;
        } else if (Math.abs(angle - TurretConstants.BARGE) < padding) {
            return State.BARGE;
        } else if (Math.abs(angle - TurretConstants.PROCESSOR_DEG) < padding) {
            return State.PROCESSOR;
        } else if (Math.abs(angle - TurretConstants.REEF_POSITION_DEG_high) < padding){
            return State.REEF_HIGH;
        } else {
            return State.IN_MOTION;
        }
    }

    public void stopMotor() {
        turretMotor.set(0);
    }


    @Override
    public void periodic() {
        double angle = getTurretAngleDeg();
        
        // Update NetworkTables
        ntAngle.setDouble(angle);
        ntPosition.setDouble(angle);
        
        // Update Mechanism2d visualization
        turretLigament.setAngle(angle);

        // Update SmartDashboard
        SmartDashboard.putString("Turret State", getState().toString());
        SmartDashboard.putNumber("Current Angle", angle);
    }

    public void setControl() {
        final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
        TrapezoidProfile.State targetState = turretTrapezoidProfile.calculate(0.02, turSetpointState, turGoalState);
        m_request.Position = targetState.position;
        m_request.Velocity = targetState.velocity;
        setSetpoint(targetState);
        turretMotor.setControl(m_request);
        turretMotor.setPosition(targetState.position);
    }

}
