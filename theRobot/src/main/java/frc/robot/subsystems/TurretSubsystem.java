// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: TurretSubsystem.java
// Intent: Control the turret aiming motor for the shooter.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.HardwareConstants;
import frc.robot.control.InstalledHardware;
import frc.robot.control.TurretAimMode;

/**
 * Controls turret angle and applies externally provided targets.
 */
public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final DigitalInput turretSensor;
    private final DigitalInput secondTurretSensor;
    private boolean hasZeroed = false;
    private boolean isAtPosition = true;
    private boolean turretSafetyOverride = true;
    // offset to add when setting position
    private double turretZeroOffsetRadians = 0.0;
    private final VoltageOut voltageOutController = new VoltageOut(0.0);

    private TurretAimMode turretAimMode = TurretAimMode.AUTO;
    private double targetTurretAngleRadians = 0.0;

    private final double minTurretAngleRadians = Math.toRadians(Constants.turretMinAngleDegrees);
    private final double maxTurretAngleRadians = Math.toRadians(Constants.turretMaxAngleDegrees);

    private double turretProfileConstraintsMaxVoltage = 11.0;
    private double turretProfileConstraintsMaxVoltageDelta = 50;
    private TrapezoidProfile.Constraints turretProfileConstraints = new TrapezoidProfile.Constraints(
            turretProfileConstraintsMaxVoltage, turretProfileConstraintsMaxVoltageDelta);
    private ProfiledPIDController turretPID = new ProfiledPIDController(0.40, 0.01, 0.00, turretProfileConstraints);
    private double minTurretVoltage = 0.40;
    private double turretLowVelocityTol = 20.0;
    private double turretPidDeadband = 0.005;

    /**
     * Create a turret subsystem with a motor a sensor
     */
    public TurretSubsystem(int turretCanId) {
        this.turretMotor = new TalonFX(turretCanId);
        this.turretSensor = InstalledHardware.turretSensorInstalled
        ? new DigitalInput(Constants.turretSensorDIOChannel)
                : null;
        this.secondTurretSensor = InstalledHardware.turretSecondSensorInstalled
        ? new DigitalInput(Constants.secondTurretSensorDIOChannel)
                : null;
        hasZeroed = turretSensor == null;
        configureMotor();
    }

    /**
     * Set the current aim mode. Manual mode holds the current turret angle.
     */
    public void setAimMode(TurretAimMode mode) {
        turretAimMode = mode;
    }

    /**
     * Get the current aim mode.
     */
    public TurretAimMode getAimMode() {
        return turretAimMode;
    }

    public void setHasZeroed(boolean hasZeroed) {
        this.hasZeroed = hasZeroed;
    }

    /**
     * Set the desired turret angle (radians) relative to the robot.
     * Normalizes rotations to [0 .. 2PI] before clamping
     * to turret min and max angles
     */
    public void setTargetAngleRadians(double turretAngleRadians) {
        targetTurretAngleRadians = clampAngleRadians(turretAngleRadians, minTurretAngleRadians, maxTurretAngleRadians);
        isAtPosition = false;
    }

    /**
     * Get the current turret angle (radians) relative to the robot.
     */
    public double getAngleRadians() {
        return getAdjustedTurretMechanismPositionRadians();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("TurretLimitSwitchTriggered", isLimitSwitchTriggered());
        SmartDashboard.putBoolean("TurretSecondLimitSwitchTriggered", isSecondLimitSwitchTriggered());
        if (!hasZeroed) {
            if (turretSensor == null) {
                hasZeroed = true;
            } else if (isLimitSwitchTriggered()) {
                stop();
                turretZeroOffsetRadians = -getRawTurretMechanismPositionRadians() + Constants.turretSensorPositionRadians;
                targetTurretAngleRadians = Constants.turretSensorPositionRadians;
                isAtPosition = true;
                hasZeroed = true;
            } else if (isSecondLimitSwitchTriggered()) {
                stop();
                turretZeroOffsetRadians = -getRawTurretMechanismPositionRadians() + Constants.turretSecondPositionRadians;
                targetTurretAngleRadians = Constants.turretSecondPositionRadians;
                isAtPosition = true;
                hasZeroed = true;
            } else {
                runVoltage(Constants.turretZeroingVoltage);
            }
        } else if (!isAtPosition){
            runVoltage(computeTurretVoltageForPosition());
            isAtPosition = isTurretWithinTolerance();
        } else { // hasZeroed and isAtPosition
            stop();
        }
        SmartDashboard.putNumber("TurretRawAngleDegrees", Math.toDegrees(getRawTurretMechanismPositionRadians()));
        SmartDashboard.putNumber("TurretAngleDegrees", Math.toDegrees(getAdjustedTurretMechanismPositionRadians()));
        SmartDashboard.putNumber("TurretTargetDegrees", Math.toDegrees(targetTurretAngleRadians));
        SmartDashboard.putBoolean("TurretIsAtPosition", isAtPosition);
    }

    protected double getAdjustedTurretMechanismPositionRadians() {
        return getRawTurretMechanismPositionRadians() + turretZeroOffsetRadians;
    }

    protected double getRawTurretMechanismPositionRadians() {
        if (RobotBase.isSimulation()) {
            return this.targetTurretAngleRadians;
        } 
        else {
            return rotationsToRadians(turretMotor.getPosition().getValueAsDouble()/Constants.turretGearRatio);
        }
    }

    public boolean isLimitSwitchTriggered() {
        return turretSensor != null && !turretSensor.get();
    }

    public boolean isSecondLimitSwitchTriggered() {
        return secondTurretSensor != null && !secondTurretSensor.get();
    }

    /**
     * A method to test whether the turret is within tolerance of the target
     * position
     * 
     * @param toleranceRadians
     * @return true if the turret is within tolerance
     */
    public boolean isTurretWithinTolerance() {
        // check both the position and velocity. To allow PID to not stop before
        // settling.
        boolean positionTargetReached = Math
                .abs(getAdjustedTurretMechanismPositionRadians() - targetTurretAngleRadians) < Constants.turretToleranceRadians;
        boolean velocityIsSmall = turretMotor == null
        || Math.abs(turretMotor.getVelocity().getValueAsDouble()) < turretLowVelocityTol;
        return positionTargetReached && velocityIsSmall;
    }


    public void runVoltage(double volts) {
        turretMotor.setControl(voltageOutController.withOutput(volts));
    }

    public void stop() {
        turretMotor.stopMotor();
        isAtPosition = true;
    }

    /**
     * clamps an angle to the closest endoint
     * @param angleRadians
     * @param minAngleRadians
     * @param maxAngleRadians
     * @return
     */
    private double clampAngleRadians(double angleRadians, double minAngleRadians, double maxAngleRadians) {
        double modAngleRadians = positiveAngleModulus(angleRadians);
        if (modAngleRadians>= minAngleRadians && modAngleRadians <= maxAngleRadians){
            return modAngleRadians;
        }
        double deltaToMin = Math.abs(MathUtil.angleModulus(modAngleRadians - minAngleRadians));
        double deltaToMax = Math.abs(MathUtil.angleModulus(modAngleRadians - maxAngleRadians));
        if (deltaToMin <= deltaToMax) {
            return minAngleRadians;
        } 
        else {
            return maxAngleRadians;
        }
    }

    /**
     * Computes a turret voltage to achieve target position using a trapezoidal PID
     */
    private double computeTurretVoltageForPosition() {
        double turretPositionRadians = getAdjustedTurretMechanismPositionRadians();
        if (!turretSafetyOverride && (turretPositionRadians < -5 || turretPositionRadians > 2 * Math.PI)){
            System.out.println("WARNING: turret at invalid position: " + turretPositionRadians + " !!!!!!!!!!!!!!!!!");
            return 0.0;
        }

        double pidOut = turretPID.calculate(turretPositionRadians, targetTurretAngleRadians);
        double out = (Math.abs(pidOut) > turretPidDeadband)
                ? pidOut + Math.signum(pidOut) * minTurretVoltage
                : 0.0;
        return out;
    }

    private void configureMotor() {
        TalonFXConfiguration talonMotorConfig = new TalonFXConfiguration();

        talonMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonMotorConfig.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        talonMotorConfig.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        talonMotorConfig.Voltage.SupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;

        talonMotorConfig.CurrentLimits.StatorCurrentLimit = HardwareConstants.ctreStatorCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimit = HardwareConstants.ctreSupplyCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // IMPORTANT! Set motor rotation so that turret runs counter clockwise positive
        // to be consistent with the direction of the robot yaw
        talonMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode response = turretMotor.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + turretMotor.getDeviceID() + " failed config with error "
                            + response.toString());
        }
    }

    /**
     * wraps an angle to [0 .. 2PI]
     * @param radians
     * @return
     */
    private double positiveAngleModulus(double angleRadians) {
        double modAngleRadians = MathUtil.angleModulus(angleRadians);
        if (modAngleRadians < 0) {
            modAngleRadians += 2 * Math.PI;
        }
        return modAngleRadians;
    }

    private double radiansToRotations(double radians){
        return radians / (2.0 * Math.PI);
    }

    private double rotationsToRadians(double rotations){
        return rotations * (2.0 * Math.PI);
    }

    /**
     * for testing and simulation only
     */
    protected void close(){
        turretSensor.close();
        secondTurretSensor.close();
    }
}