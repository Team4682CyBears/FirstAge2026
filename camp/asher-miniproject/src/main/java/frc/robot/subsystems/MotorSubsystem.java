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

import frc.robot.Constants;

public class MotorSubsystem extends SubsystemBase{
    //I stole magic numbers from ted
    private final TalonFX motor;
        private final DutyCycleOut eeDutyCycle = new DutyCycleOut(0.0);

    private final CANcoder motorEncoder;


    public MotorSubsystem(int motorCan, int encoderCan){
        this.motor = new TalonFX(motorCan);
        configureMotor();

        this.motorEncoder = new CANcoder(encoderCan);
        configureAngleEncoder();
    }

    public void setMotorSpeed(double motorSpeed){
        eeDutyCycle.withOutput(clamp(motorSpeed, -1, 1));
        motor.setControl(eeDutyCycle);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Get the current position of the endodger in degrees.
     */
    public double getEndodgerPositionDegrees(){
        return wrapDegrees(motorEncoder.getPosition().getValueAsDouble()*360);
    }

    public static double clamp(double value, double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min must not be greater than max");
        }

        return max(min, min(value, max));
    }

    private void configureAngleEncoder() {
        // Config CanCoder
        CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
        encoderConfigs.MagnetSensor.MagnetOffset = degreesToRotations(Constants.encoderAbsoluteOffsetDegrees);
        encoderConfigs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
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
        return (degrees % 360 + 540) % 360 - 180;
    }
}