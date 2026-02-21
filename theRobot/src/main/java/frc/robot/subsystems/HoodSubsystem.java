package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

public class HoodSubsystem extends SubsystemBase {

    // Hood gearing
    private static final double hoodEncoderGearRatio = 1.0;
    private static final double hoodExtendoLowVelocityTol = 10; // TODO test this on device with motion magic profile

    private TalonFXS motor;
    private CANcoder encoder;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0);

    private boolean hoodIsAtDesiredExtension = true;
    private double desiredExtension;

    private Slot0Configs hoodMotorGainsForAbsoluteEncoder = new Slot0Configs().withKP(150).withKI(0.125).withKD(0.0)
            .withKV(0.1).withKS(0.1); // TODO: Find real values. DO NOT SET KD!!

    public HoodSubsystem() {
        if (InstalledHardware.hoodEncoderInstalled) {
            this.encoder = new CANcoder(Constants.hoodEncoderCanID);
            configureEncoder();
        }

        if (InstalledHardware.hoodMotorInstalled) {
            this.motor = new TalonFXS(Constants.hoodMotorCanID);
            configureMotor();
        }
    }

    public void setExtendoPosition(double position) {
        desiredExtension = MathUtil.clamp(position, Constants.hoodMinPositionRotations,
                Constants.hoodMaxPositionRotations);
        hoodIsAtDesiredExtension = false;
    }

    private void configureEncoder() {
        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
        ccConfig.MagnetSensor.MagnetOffset = Constants.hoodEncoderAbsoluteOffset;
        ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // apply configs
        StatusCode response = encoder.getConfigurator().apply(ccConfig);
        if (!response.isOK()) {
            DataLogManager.log(
                    "CANcoder ID " + encoder.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    /**
     * A method to get the hood extendo
     * 
     * @return hood extendo in rotations
     */
    public double getHoodPosition() {
        return InstalledHardware.hoodEncoderInstalled ? encoder.getPosition().getValueAsDouble()
                : motor.getPosition().getValueAsDouble();
    }

    /**
     * this method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        if (!hoodIsAtDesiredExtension) {
            // use motionMagic voltage control
            motor.setControl(
                    voltageController.withPosition(desiredExtension));
            // keep moving until it reaches target extendo
            hoodIsAtDesiredExtension = isExtendoWithinTolerance(desiredExtension);
        }
        SmartDashboard.putNumber("Hood Absolute Position",
                encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Motor Encoder Extendo", getHoodPosition());
    }

    /**
     * A method to test whether the extendo is within tolerance of the target
     * extendo
     * 
     * @param targetExtendoTolerance
     * @return true if the extendo is within tolerance
     */
    public boolean isExtendoWithinTolerance(double targetExtendoTolerance) {
        // check both the position and velocity. To allow PID to not stop before
        // settling.
        boolean positionTargetReached = Math
                .abs(getHoodPosition() - desiredExtension) < Constants.hoodExtendoTolerance;
        boolean velocityIsSmall = Math.abs(motor.getVelocity().getValueAsDouble()) < hoodExtendoLowVelocityTol;
        return positionTargetReached && velocityIsSmall;
    }

    private void configureMotor() {
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        config.ExternalFeedback.FeedbackRemoteSensorID = Constants.hoodEncoderCanID;
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.RemoteCANcoder;
        config.Slot0 = hoodMotorGainsForAbsoluteEncoder;
        config.ExternalFeedback.SensorToMechanismRatio = hoodEncoderGearRatio;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        config.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        config.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Borrowed from Crescendo2024 ShooterAngleSubsystem.java
        // TODO: Verify values
        config.MotionMagic.MotionMagicCruiseVelocity = 800.0;
        config.MotionMagic.MotionMagicAcceleration = 160;
        config.MotionMagic.MotionMagicJerk = 800;

        // Software limit switches
        config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Constants.hoodMaxPositionRotations)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.hoodMinPositionRotations);

        StatusCode response = motor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
        }
    }
}
