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

    private final PositionVoltage positionController = new PositionVoltage(0.0);
    private final VoltageOut voltageOutController = new VoltageOut(0.0);

    private TurretAimMode turretAimMode = TurretAimMode.AUTO;
    private double targetTurretAngleRadians = 0.0;

    private final double minTurretAngleRadians = Math.toRadians(Constants.turretMinAngleDegrees);
    private final double maxTurretAngleRadians = Math.toRadians(Constants.turretMaxAngleDegrees);
    private final Rotation2d turretZeroOffset = Constants.turretZeroOffsetDegrees;

    // TODO tune with robot-on-carpet data
    private final Slot0Configs slot0Configs = new Slot0Configs()
        .withKP(12.0)
        .withKI(0.0)
        .withKD(0.4)
        .withKS(.1)
        .withKV(.15);

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
        if (mode == TurretAimMode.MANUAL) {
            targetTurretAngleRadians = MathUtil.clamp(getTurretAngleRadiansContinuous(),
                    minTurretAngleRadians, maxTurretAngleRadians);
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
    targetTurretAngleRadians = MathUtil.clamp(turretAngleRadians, minTurretAngleRadians,
        maxTurretAngleRadians);
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
        targetTurretAngleRadians = turretZeroOffset.getRadians();
    }

    @Override
    public void periodic() {
        if (!hasZeroed) {
            if (turretSensor == null) {
                System.out.println("No Sensor!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                hasZeroed = true;
            } else if (isLimitSwitchTriggered()) {
                turretMotor.setPosition(Constants.turretSensorPosition.getRotations());
                targetTurretAngleRadians = getTurretAngleRadiansContinuous();
                stop();
                hasZeroed = true;
                System.out.println("Turret zeroing complete.!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            } else {
                System.out.println("Turret zeroing in progress...!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                runVoltage(Constants.turretZeroingVoltage);
            }
        } else {

        double targetTurretRotations = (targetTurretAngleRadians - turretZeroOffset.getRadians())
                / (2.0 * Math.PI * Constants.turretAngleSign);
        positionController.withPosition(targetTurretRotations);
        turretMotor.setControl(positionController);

        SmartDashboard.putNumber("TurretAngleDegrees", getAngleRadians().getDegrees());
        SmartDashboard.putNumber("TurretTargetDegrees", Math.toDegrees(targetTurretAngleRadians));
        }
    }

    private double getTurretMechanismAngleRadians() {
        return turretMotor.getPosition().getValueAsDouble() * 2.0 * Math.PI * Constants.turretAngleSign;
    }

    private double getTurretAngleRadiansContinuous() {
        return getTurretMechanismAngleRadians() + turretZeroOffset.getRadians();
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
        targetTurretAngleRadians = MathUtil.clamp(getTurretAngleRadiansContinuous(),
                minTurretAngleRadians, maxTurretAngleRadians);
        turretMotor.setControl(voltageOutController.withOutput(0.0));
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