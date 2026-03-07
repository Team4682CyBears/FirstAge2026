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
import frc.robot.control.InstalledHardware;

public class IntakeWristSubsystem extends SubsystemBase {

    // wrist gearing
    private static final double intakeWristEncoderGearRatio = 1.0/5.0 * 1.0/5.0 * 22.0/32.0;
    private static final double intakeWristLowVelocityTol = 10; // TODO test this on device with motion magic profile

    private TalonFX motor;
    private CANcoder encoder;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0);

    private boolean intakeWristIsAtDesiredExtension = true;
    private double desiredExtension = 0.0;

    private Slot0Configs slot0Configs = new Slot0Configs().withKP(0.4).withKI(0.0).withKD(0.0).withKG(0.2)
            .withKV(0.45045).withKS(0.09009); // TODO: Find real values. DO NOT SET KD!!

    public IntakeWristSubsystem(int motorCanID, int encoderCanID) {
        // Only create/configure encoder if hardware is present
        if (InstalledHardware.intakeWristEncoderInstalled) {
            this.encoder = new CANcoder(encoderCanID);
            configureEncoder();
        } else {
            this.encoder = null;
        }

        if (InstalledHardware.intakeWristMotorInstalled) {
            this.motor = new TalonFX(motorCanID);
            configureMotor();
        }
    }

    public void setExtendoPosition(double position) {
        desiredExtension = MathUtil.clamp(position, Constants.intakeWristRetractedPositionRotations,
                Constants.intakeWristDeployedPositionRotations);
        intakeWristIsAtDesiredExtension = false;
    }

    /**
     * A method to get the wrist position
     * 
     * @return wrist position in rotations
     */
    public double getWristPosition() {
        if (encoder != null) {
        return encoder.getPosition().getValueAsDouble();
        }
        return motor.getPosition().getValueAsDouble();
    }

    public double getDesiredExtension() {
        return desiredExtension;
    }

    /**
     * this method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        intakeWristIsAtDesiredExtension = isExtendoWithinTolerance();
        if (!intakeWristIsAtDesiredExtension){
        motor.setControl(voltageController.withPosition(desiredExtension));
        }
        if (encoder != null) {
            SmartDashboard.putNumber("IntakeWrist Absolute Position", encoder.getPosition().getValueAsDouble());
        } else {
            SmartDashboard.putNumber("IntakeWrist Absolute Position", Double.NaN);
        }
        SmartDashboard.putNumber("IntakeWrist Motor Encoder Extendo", getWristPosition());
    }

    /**
     * A method to test whether the extendo is within tolerance of the target
     * extendo
     * 
     * @return true if the extendo is within tolerance
     */
    public boolean isExtendoWithinTolerance() {
        // check both the position and velocity. To allow PID to not stop before
        // settling.
        boolean positionTargetReached = Math
                .abs(getWristPosition() - desiredExtension) < Constants.intakeWristTolerance;
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
        //TODO when we have an encoder, we will need to put the motor config back
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Slot0 = slot0Configs;
        config.Feedback.SensorToMechanismRatio = 1.0/intakeWristEncoderGearRatio;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        config.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        config.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Borrowed from Crescendo2024 ShooterAngleSubsystem.java
        // TODO: Verify values
        config.MotionMagic.MotionMagicCruiseVelocity = 800.0;
        config.MotionMagic.MotionMagicAcceleration = 160;
        config.MotionMagic.MotionMagicJerk = 800;

        // Software limit switches
        // forward = maximum allowed extension, reverse = minimum.  the
        // previous configuration had these swapped which meant the controller
        // would immediately hit a soft limit as soon as it tried to move
        // toward the retracted position and then hold against the "wrong"
        // limit; that combined with the inverted motor produced the behaviour
        // described by the driver.  swap them so the limits reflect the
        // physical positions in Constants.
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
