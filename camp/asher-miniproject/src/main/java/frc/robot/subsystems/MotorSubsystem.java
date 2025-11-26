// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: MotorSubsystem.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static java.lang.Math.max;
import static java.lang.Math.min;

import edu.wpi.first.math.MathUtil.*;

import frc.robot.Constants;

/**
 * The MotorSubsystem is responsible for controlling a motor and its associated encoder.
 * It provides methods to set motor speed, retrieve the position of the encoder, and configure
 * the motor and encoder settings.
 *
 * <p>This subsystem uses a TalonFX motor controller and a CANcoder encoder. The motor speed
 * is clamped to a range of -1 to 1, and the encoder position is returned in degrees.
 *
 * <p>Key features:
 * - Configures motor settings such as neutral mode, voltage limits, and current limits.
 * - Configures encoder settings such as magnet offset and sensor direction.
 * - Provides utility methods for clamping values and wrapping degrees.
 */
public class MotorSubsystem extends SubsystemBase{
    private final TalonFX motor;
    private final DutyCycleOut eeDutyCycle = new DutyCycleOut(0.0);

    private final CANcoder motorEncoder;


    /**
     * Constructs a MotorSubsystem with a motor and an encoder.
     *
     * @param motorCan The CAN ID of the motor controller (TalonFX).
     * @param encoderCan The CAN ID of the encoder (CANcoder).
     */
    public MotorSubsystem(int motorCan, int encoderCan){
        this.motor = new TalonFX(motorCan);
        configureMotor();

        this.motorEncoder = new CANcoder(encoderCan);
        configureAngleEncoder();
    }

    /**
     * Sets the speed of the motor within the allowed range of -1 to 1.
     *
     * @param motorSpeed The desired speed of the motor. Values are clamped between -1 (full reverse) and 1 (full forward).
     */
    public void setMotorSpeed(double motorSpeed){
        if(motorSpeed == 0){
            motor.stopMotor();
        } else {
            eeDutyCycle.withOutput(MathUtil.clamp(motorSpeed, -1, 1));
            motor.setControl(eeDutyCycle);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }


    /**
     * Retrieves the position of the endodger in degrees.
     * <p>
     * This method calculates the position of the endodger by converting the
     * encoder's position value (in rotations) to degrees and wrapping it
     * within a valid range of degrees (e.g., 0 to 360).
     * </p>
     *
     * @return The position of the endodger in degrees, wrapped within a valid range.
     */
    public double getEndodgerPositionDegrees(){
        return wrapDegrees(motorEncoder.getPosition().getValueAsDouble()*360);
    }

    private void configureAngleEncoder() {
        // Config CanCoder
        CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
        encoderConfigs.MagnetSensor.MagnetOffset = degreesToRotations(Constants.encoderAbsoluteOffsetDegrees);
        encoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = Constants.absoluteSensorDiscontinuityPoint;
        encoderConfigs.MagnetSensor.SensorDirection = Constants.shooterAngleSensorDirection;
        // apply configs
        StatusCode response = motorEncoder.getConfigurator().apply(encoderConfigs);
        if (!response.isOK()) {
        DataLogManager.log(
            "CANcoder ID " + motorEncoder.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        config.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        config.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

        config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode response = motor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    private double degreesToRotations(double degrees)
    {
      return degrees/360;
    }

    private double wrapDegrees(double degrees){
        return (degrees + 540) % 360 - 180;
    }
}