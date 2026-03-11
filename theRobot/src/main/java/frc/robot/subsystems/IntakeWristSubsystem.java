// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: IntakeWrist.jave
// Intent: Retracts and deploys the intake
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ExternalFeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.control.IntakeWristMode;

public class IntakeWristSubsystem extends SubsystemBase {

    // wrist gearing
    private static final double intakeWristRotorToSensorRatio = 1.0/5.0 * 1.0/5.0;
    private static final double intakeWristSensorToMechanismRatio = 18.0/32.0;
    private static final double intakeWristLowVelocityTol = 10;

    private TalonFX motor;
    private CANcoder encoder;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0);

    private boolean intakeWristIsAtDesiredExtension = true;
    private IntakeWristMode intakeWristMode = IntakeWristMode.RETRACTED;
    private double desiredExtension = Constants.intakeWristRetractedPositionRotations;

    private Slot0Configs slot0Configs = new Slot0Configs().withKP(0.36).withKI(0.003).withKD(0.0).withKG(0.24)
            .withKV(3.85).withKS(0.5).withGravityType(GravityTypeValue.Arm_Cosine); // DO NOT SET KD!!

    public IntakeWristSubsystem(int motorCanID, int encoderCanID) {
        if (InstalledHardware.intakeWristEncoderInstalled) {
            this.encoder = new CANcoder(encoderCanID);
            configureEncoder();
        }
        if (InstalledHardware.intakeWristMotorInstalled) {
            this.motor = new TalonFX(motorCanID);
            configureMotor();
        }
        // this has to be done last
        if (!InstalledHardware.intakeWristEncoderInstalled){
            // wrist starts retracted
            this.motor.setPosition(Constants.intakeWristRetractedPositionRotations);
        }
    }

    /**
     * A method to get the wrist mode
     * @return wrist mode
     */
    public IntakeWristMode getMode(){
        return intakeWristMode;
    }

    /**
     * A method to get the wrist position
     * 
     * @return wrist position in rotations
     */
    public double getPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    /**
     * this method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        intakeWristIsAtDesiredExtension = isPositionWithinTolerance();
        if (!intakeWristIsAtDesiredExtension){
            motor.setControl(voltageController.withPosition(desiredExtension));
        }
        SmartDashboard.putNumber("IntakeWrist Motor Encoder Position", getPosition());
    }

    /**
     * A method to set the wrist position
     * @param position
     */
    public void setPosition(double position) {
        desiredExtension = MathUtil.clamp(position, Constants.intakeWristDeployedPositionRotations,
                Constants.intakeWristRetractedPositionRotations);
        intakeWristIsAtDesiredExtension = false;
    }

    public void setMode(IntakeWristMode mode){
        if (mode == intakeWristMode){
            // already in this mode
            return;
        } 
        else {
            intakeWristMode = mode;
            switch (mode) {
                case DEPLOYED:
                    setPosition(Constants.intakeWristDeployedPositionRotations);
                    break;
                case RETRACTED:
                    setPosition(Constants.intakeWristRetractedPositionRotations);
                    break;
                default:
                    setPosition(Constants.intakeWristRetractedPositionRotations);
            }
        }
    }

    private void configureEncoder() {
        CANcoderConfiguration ccConfig = new CANcoderConfiguration();
        ccConfig.MagnetSensor.MagnetOffset = Constants.intakeWristEncoderAbsoluteOffset;
        // sensor range is (0 .. 0.25) so setting this to 0.9 to be outside the viable range
        ccConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9;
    
        ccConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        // apply configs
        StatusCode response = encoder.getConfigurator().apply(ccConfig);
        if (!response.isOK()) {
            DataLogManager.log(
                    "CANcoder ID " + encoder.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        if (!InstalledHardware.intakeWristEncoderInstalled) {
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            // when using the internal sensor, configure the gear ratio between the sensor to mechanism.
            config.Feedback.SensorToMechanismRatio = 1.0/(intakeWristRotorToSensorRatio * intakeWristSensorToMechanismRatio);
        } else {
            config.Feedback.FeedbackRemoteSensorID = Constants.intakeWristEncoderCanID;
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;  
            config.Feedback.SensorToMechanismRatio = 1.0 / intakeWristSensorToMechanismRatio;
            // when using the external sensor, configure the gear ratio between the rotor to sensor. 
            config.Feedback.RotorToSensorRatio = 1.0 / intakeWristRotorToSensorRatio;
        }

        config.Slot0 = slot0Configs;
        config.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.02);

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
        config.MotionMagic.MotionMagicCruiseVelocity = 800.0;
        config.MotionMagic.MotionMagicAcceleration = 160;
        config.MotionMagic.MotionMagicJerk = 800;

        StatusCode response = motor.getConfigurator().apply(config);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    /**
     * A method to test whether the position is within tolerance of the target
     * position
     * 
     * @return true if the position is within tolerance
     */
    private boolean isPositionWithinTolerance() {
        //if deployed never asume within tolerance to keep driving
        if (intakeWristMode == IntakeWristMode.DEPLOYED){
            return false;
        }
        // check both the position and velocity. To allow PID to not stop before
        // settling.
        boolean positionTargetReached = Math
                .abs(getPosition() - desiredExtension) < Constants.intakeWristTolerance;
        boolean velocityIsSmall = Math.abs(motor.getVelocity().getValueAsDouble()) < intakeWristLowVelocityTol;
        return positionTargetReached && velocityIsSmall;
    }

}
