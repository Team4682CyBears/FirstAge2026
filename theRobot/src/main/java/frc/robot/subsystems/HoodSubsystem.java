// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: HoodSubsystem.java
// Intent: runs the talonfxs motor to extend the hood
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

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
import com.ctre.phoenix6.signals.MotorArrangementValue;
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
    private static final double hoodMotorToMechanismGearRatio = (12.0 / 29.0) * (14.0 / 27.0);
    private static final double hoodExtendoLowVelocityTol = 10; // TODO test this on device with motion magic profile

    private TalonFXS motor;
    private CANcoder encoder;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0);

    private boolean hoodIsAtDesiredExtension = true;
    private double desiredExtension;

    private Slot0Configs hoodMotorGainsForAbsoluteEncoder = new Slot0Configs().withKP(0.4).withKI(0.01).withKD(0.0)
            .withKV(0.495).withKS(0.1).withKG(0.02); 

    public HoodSubsystem(int hoodMotorCanID, int hoodEncoderID) {
        if (InstalledHardware.hoodEncoderInstalled) {
            this.encoder = new CANcoder(hoodEncoderID);
            configureEncoder();
        }

        if (InstalledHardware.hoodMotorInstalled) {
            this.motor = new TalonFXS(hoodMotorCanID);
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
        // sensor range is (0 .. 0.635) so setting this to 0.9 to be outside the viable range
        ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
    
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
        return InstalledHardware.hoodEncoderInstalled ? encoder.getAbsolutePosition().getValueAsDouble()
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
        SmartDashboard.putBoolean("Hood Within Tolerance", hoodIsAtDesiredExtension);
        SmartDashboard.putNumber("Hood Absolute Position",
                encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Hood Motor Encoder Extendo", motor.getPosition().getValueAsDouble());
    }

    /**
     * @return true if the hood has reached (and is settled at) the most
     *         recently requested position.
     */
    public boolean isAtDesiredPosition() {
        return hoodIsAtDesiredExtension;
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
        config.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.SyncCANcoder;
        config.Slot0 = hoodMotorGainsForAbsoluteEncoder;
        config.ExternalFeedback.SensorToMechanismRatio = 1.0;
        config.ExternalFeedback.RotorToSensorRatio = 1.0 / hoodMotorToMechanismGearRatio;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        config.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        config.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        // TODO: Verify values
        config.MotionMagic.MotionMagicCruiseVelocity = 800.0;
        config.MotionMagic.MotionMagicAcceleration = 160.0;
        config.MotionMagic.MotionMagicJerk = 800.0;

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
