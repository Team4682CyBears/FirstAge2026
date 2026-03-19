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
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private boolean lastSensorDetected = false;

    private final PositionVoltage positionController = new PositionVoltage(0.0);

    private TurretAimMode turretAimMode = TurretAimMode.AUTO;
    private Rotation2d targetTurretAngle = new Rotation2d();

    private final Rotation2d minTurretAngle = Constants.turretMinAngleDegrees;
    private final Rotation2d maxTurretAngle = Constants.turretMaxAngleDegrees;
    private final Rotation2d turretZeroOffset = Constants.turretZeroOffsetDegrees;

    // TODO tune with robot-on-carpet data
    private static final double turretPositionKp = 12.0;
    private static final double turretPositionKi = 0.0;
    private static final double turretPositionKd = 0.4;

    // TODO tune with robot-on-carpet data
    private final Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(turretPositionKp)
            .withKI(turretPositionKi)
            .withKD(turretPositionKd);

    /**
     * Create a turret subsystem with a motor a sensor.
     */
    public TurretSubsystem(int turretCanId) {
        this.turretMotor = new TalonFX(turretCanId);
        this.turretSensor = InstalledHardware.turretSensorInstalled
        ? new DigitalInput(Constants.turretSensorChannel)
                : null;
        if (turretSensor != null) {
            lastSensorDetected = turretSensor.get();
        }
        configureMotor();
    }

    /**
     * Set the current aim mode. Manual mode holds the current turret angle.
     */
    public void setAimMode(TurretAimMode mode) {
        if (mode == TurretAimMode.MANUAL) {
            targetTurretAngle = getAngleRadians();
        }
        turretAimMode = mode;
    }

    /**
     * Get the current aim mode.
     */
    public TurretAimMode getAimMode() {
        return turretAimMode;
    }

    /**
     * Set the desired turret angle (radians) relative to the robot.
     */
    public void setTargetAngleRadians(double turretAngleRadians) {
        double clampedRadians = MathUtil.clamp(turretAngleRadians, minTurretAngle.getRadians(),
                maxTurretAngle.getRadians());
        targetTurretAngle = Rotation2d.fromRadians(clampedRadians);
    }

    /**
     * Get the current turret angle (radians) relative to the robot.
     */
    public Rotation2d getAngleRadians() {
        return Rotation2d.fromRadians(
                MathUtil.angleModulus(getTurretMechanismAngleRadians() + turretZeroOffset.getRadians()));
    }

    /**
     * Reset turret encoder position to match the zero offset.
     */
    public void zeroTurret() {
        turretMotor.setPosition(0.0);
        targetTurretAngle = Rotation2d.fromRadians(MathUtil.angleModulus(turretZeroOffset.getRadians()));
    }

    @Override
    public void periodic() {
        updateTurretZeroFromSensor();
    double targetTurretRotations = targetTurretAngle.minus(turretZeroOffset).getRotations();
        positionController.withPosition(targetTurretRotations);
        turretMotor.setControl(positionController);

        SmartDashboard.putNumber("TurretAngleDegrees", getAngleRadians().getDegrees());
        SmartDashboard.putNumber("TurretTargetDegrees", targetTurretAngle.getDegrees());
    }

    private double getTurretMechanismAngleRadians() {
        Rotation2d turretRotations = Rotation2d.fromRotations(turretMotor.getPosition().getValueAsDouble());
        return turretRotations.getRadians() * Constants.turretAngleSign;
    }

    private void updateTurretZeroFromSensor() {
        if (turretSensor == null) {
            return;
        }

        boolean detected = turretSensor.get();
        if (detected != lastSensorDetected) {
            double velocity = turretMotor.getVelocity().getValueAsDouble();
            boolean movingUp = velocity > 0.0;
            boolean movingDown = velocity < 0.0;

            if ((movingDown && detected) || (movingUp && !detected)) {
                turretMotor.setPosition(Constants.turretSensorPosition.getRotations());
                targetTurretAngle = Constants.turretSensorPosition;
            }
        }

        lastSensorDetected = detected;
    }

    private void configureMotor() {
        TalonFXConfiguration talonMotorConfig = new TalonFXConfiguration();
        
        talonMotorConfig.Feedback.SensorToMechanismRatio = Constants.turretGearRatio;

        talonMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonMotorConfig.Slot0 = slot0Configs;
        talonMotorConfig.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        talonMotorConfig.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        talonMotorConfig.Voltage.SupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;

        talonMotorConfig.CurrentLimits.StatorCurrentLimit = HardwareConstants.ctreStatorCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimit = HardwareConstants.ctreSupplyCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        talonMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxTurretAngle.getRotations();
    talonMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minTurretAngle.getRotations();

        StatusCode response = turretMotor.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + turretMotor.getDeviceID() + " failed config with error "
                            + response.toString());
        }
    }
}