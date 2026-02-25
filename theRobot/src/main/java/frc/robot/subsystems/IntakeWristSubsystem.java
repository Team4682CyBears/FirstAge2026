package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;

public class IntakeWristSubsystem extends SubsystemBase {

    // wrist gearing
    private static final double intakeWristEncoderGearRatio = 1.0;
    private static final double intakeWristLowVelocityTol = 10; // TODO test this on device with motion magic profile

    private TalonFX motor;
    private CANcoder encoder;
    private int encodeCanID;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0);

    private boolean intakeWristIsAtDesiredExtension = true;
    private double desiredExtension;

    private Slot0Configs slot0Configs = new Slot0Configs().withKP(150).withKI(0.125).withKD(0.0)
            .withKV(0.1).withKS(0.1); // TODO: Find real values. DO NOT SET KD!!

    public IntakeWristSubsystem(int motorCanID, int encoderCanID) {
        this.motor = new TalonFX(motorCanID);
        this.encodeCanID = encoderCanID;

        // Only create/configure encoder if hardware is present
        if (frc.robot.control.InstalledHardware.intakeWristEncoderInstalled) {
            this.encoder = new CANcoder(encoderCanID);
            configureEncoder();
        } else {
            this.encoder = null;
        }

        configureMotor();
    }

    public void setExtendoPosition(double position) {
        desiredExtension = MathUtil.clamp(position, Constants.intakeWristRetractedPositionRotations,
                Constants.intakeWristDeployedPositionRotations);
        intakeWristIsAtDesiredExtension = false;
    }

    /**
     * A method to get the hood extendo
     * 
     * @return hood extendo in rotations
     */
    public double getHoodPosition() {
        if (encoder != null) {
        return encoder.getPosition().getValueAsDouble();
        }
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * this method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        motor.setControl(voltageController.withPosition(desiredExtension));
        intakeWristIsAtDesiredExtension = isExtendoWithinTolerance(desiredExtension);
        
        if (encoder != null) {
            SmartDashboard.putNumber("IntakeWrist Absolute Position", encoder.getPosition().getValueAsDouble());
        } else {
            SmartDashboard.putNumber("IntakeWrist Absolute Position", Double.NaN);
        }
        SmartDashboard.putNumber("IntakeWrist Motor Encoder Extendo", getHoodPosition());
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
                .abs(getHoodPosition() - desiredExtension) < Constants.intakeWristTolerance;
        boolean velocityIsSmall = Math.abs(motor.getVelocity().getValueAsDouble()) < intakeWristLowVelocityTol;
        return positionTargetReached && velocityIsSmall;
    }

    private void configureEncoder() {
        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
    ccConfig.MagnetSensor.MagnetOffset = Constants.intakeWristEncoderAbsoluteOffset;
    ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // apply configs
        StatusCode response = encoder.getConfigurator().apply(ccConfig);
        if (!response.isOK()) {
            DataLogManager.log(
                    "CANcoder ID " + encoder.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        if (encoder != null) {
            config.Feedback.FeedbackRemoteSensorID = encodeCanID;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        }
        config.Slot0 = slot0Configs;
        config.Feedback.SensorToMechanismRatio = intakeWristEncoderGearRatio;

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
        .withForwardSoftLimitThreshold(Constants.intakeWristDeployedPositionRotations)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(Constants.intakeWristRetractedPositionRotations);

        StatusCode response = motor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
        }
    }
}
