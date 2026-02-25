package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

public class IntakeWristSubsystem extends SubsystemBase {

    // Hood gearing
    private static final double intakeWristEncoderGearRatio = 1.0;
    private static final double intakeWristExtendoLowVelocityTol = 10; // TODO test this on device with motion magic profile

    private TalonFXS motor;
    private CANcoder encoder;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0);

    private boolean intakeWristIsAtDesiredExtension = true;
    private double desiredExtension;

    private Slot0Configs slot0Configs = new Slot0Configs().withKP(150).withKI(0.125).withKD(0.0)
            .withKV(0.1).withKS(0.1); // TODO: Find real values. DO NOT SET KD!!

    public IntakeWristSubsystem() {
        this.encoder = new CANcoder(Constants.IntakeWristCanID);
        configureEncoder();
    }

    public void setExtendoPosition(double position) {
        desiredExtension = MathUtil.clamp(position, Constants.intakeWristMinPositionRotations,
                Constants.intakeWristMaxPositionRotations);
        intakeWristIsAtDesiredExtension = false;
    }

    private void configureEncoder() {
        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
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
            return motor.getPosition().getValueAsDouble();
    }

    /**
     * this method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        if (!intakeWristIsAtDesiredExtension) {
            // use motionMagic voltage control
            motor.setControl(
                    voltageController.withPosition(desiredExtension));
            // keep moving until it reaches target extendo
            intakeWristIsAtDesiredExtension = isExtendoWithinTolerance(desiredExtension);
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
                .abs(getHoodPosition() - desiredExtension) < Constants.intakeWristExtendoTolerance;
        boolean velocityIsSmall = Math.abs(motor.getVelocity().getValueAsDouble()) < intakeWristExtendoLowVelocityTol;
        return positionTargetReached && velocityIsSmall;
    }

    private void configureMotor() {
        TalonFXConfiguration talonMotorConfig = new TalonFXConfiguration();
        talonMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonMotorConfig.Slot0 = slot0Configs;
        talonMotorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.02);
        // do not config feedbacksource, since the default is the internal one.
        talonMotorConfig.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;

        talonMotorConfig.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        talonMotorConfig.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        // maximum current settings
        talonMotorConfig.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        talonMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Borrowed from Crescendo2024 ShooterAngleSubsystem.java
        // TODO: Verify values
        talonMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 800.0;
        talonMotorConfig.MotionMagic.MotionMagicAcceleration = 160;
        talonMotorConfig.MotionMagic.MotionMagicJerk = 800;

        // Software limit switches
        talonMotorConfig.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitEnable(true)
                .withForwardSoftLimitThreshold(Constants.intakeWristMaxPositionRotations)
                .withReverseSoftLimitEnable(true)
                .withReverseSoftLimitThreshold(Constants.intakeWristMinPositionRotations);

        /*StatusCode response = motor.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
        }*/
    }
}
