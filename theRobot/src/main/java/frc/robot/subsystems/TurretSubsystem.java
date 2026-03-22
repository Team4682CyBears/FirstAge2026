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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
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
    private boolean hasZeroed = false;
    private final PositionVoltage positionController = new PositionVoltage(0.0)
        .withFeedForward(.7);
    private final VoltageOut voltageOutController = new VoltageOut(0.0);

    private TurretAimMode turretAimMode = TurretAimMode.AUTO;
    private double targetTurretAngleRadians = 0.0;
    // adjusted is target + offset

    private final double minTurretAngleRadians = Math.toRadians(Constants.turretMinAngleDegrees);
    private final double maxTurretAngleRadians = Math.toRadians(Constants.turretMaxAngleDegrees);
    private final Rotation2d turretZeroOffset = Constants.turretZeroOffsetDegrees;

    // TODO tune with robot-on-carpet data
    private final Slot0Configs slot0Configs = new Slot0Configs()
        .withKP(9.5)
        .withKI(0.0)
        .withKD(0.4)
        .withKI(.01);

    /**
     * Create a turret subsystem with a motor a sensor.
     */
    public TurretSubsystem(int turretCanId) {
        this.turretMotor = new TalonFX(turretCanId);
        this.turretSensor = InstalledHardware.turretSensorInstalled
        ? new DigitalInput(Constants.turretSensorDIOChannel)
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

    /**
     * Set the desired turret angle (radians) relative to the robot.
     */
    public void setTargetAngleRadians(double turretAngleRadians) {
        targetTurretAngleRadians = turretAngleRadians;
    }

    /**
     * Get the current turret angle (radians) relative to the robot.
     */
    public Rotation2d getAngleRotation2d() {
        // this has the offset taken out. Do not use internally. Use getTurretMechanismAngleRadians instead
        return Rotation2d.fromRadians(
                MathUtil.angleModulus(getTurretMechanismAngleRadians() - turretZeroOffset.getRadians()));
    }

    /**
     * Reset turret encoder position to match the zero offset.
     */
    public void zeroTurret() {
        turretMotor.setPosition(0.0);
        targetTurretAngleRadians = getAngleRotation2d().getRadians();
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("TurretLimitSwitchTriggered", isLimitSwitchTriggered());
        if (!hasZeroed) {
            if (turretSensor == null) {
                hasZeroed = true;
            } else if (isLimitSwitchTriggered()) {
                turretMotor.setPosition(Constants.turretSensorPosition.getRotations());
                stop();
                hasZeroed = true;
            } else {
                runVoltage(Constants.turretZeroingVoltage);
            }
        } else {

        double adjustedTurretRotations = MathUtil.clamp((MathUtil.angleModulus(targetTurretAngleRadians + turretZeroOffset.getRadians()) + Math.PI)
                / (2.0 * Math.PI * Constants.turretAngleSign), minTurretAngleRadians, maxTurretAngleRadians);
        positionController.withPosition(adjustedTurretRotations);
        turretMotor.setControl(positionController);

        SmartDashboard.putNumber("TurretAngleDegrees", getAngleRotation2d().getDegrees());
        SmartDashboard.putNumber("TurretTargetDegrees", Math.toDegrees(targetTurretAngleRadians));
        }
    }

    private double getTurretMechanismAngleRadians() {
        return turretMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI * Constants.turretAngleSign;
    }

    public boolean isLimitSwitchAvailable() {
        return turretSensor != null;
    }

    public boolean isLimitSwitchTriggered() {
        return turretSensor != null && !turretSensor.get();
    }

    public void runVoltage(double volts) {
        turretMotor.setControl(voltageOutController.withOutput(volts));
    }

    public void stop() {
        targetTurretAngleRadians = getAngleRotation2d().getRadians();
        turretMotor.stopMotor();
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

        //talonMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        //talonMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    //talonMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = maxTurretAngleRadians / (2.0 * Math.PI);
    //talonMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = minTurretAngleRadians / (2.0 * Math.PI);

        StatusCode response = turretMotor.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + turretMotor.getDeviceID() + " failed config with error "
                            + response.toString());
        }
    }
}