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
import com.ctre.phoenix6.controls.VoltageOut;

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
import frc.robot.control.IntakeWristMode;

public class IntakeWristSubsystem extends SubsystemBase {

    // wrist gearing
    private static final double intakeWristRotorToSensorRatio = 1.0/5.0 * 1.0/5.0;
    private static final double intakeWristSensorToMechanismRatio = 18.0/32.0;
    private static final double intakeWristLowVelocityTol = 10;
    // if using voltageOut control, here's the values you would use for forward and reverse.
    public static final double intakeWristForwardVoltage = 1.9;
    public static final double intakeWristReverseVoltage = -1.8;

    private TalonFX motor;
    private CANcoder encoder;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0).withOverrideBrakeDurNeutral(true);
    // For manual mode;
    private boolean isManualMode = false;
    private VoltageOut voltageOut = new VoltageOut(0.0);

    private boolean intakeWristIsAtDesiredExtension = true;
    private IntakeWristMode intakeWristMode = IntakeWristMode.RETRACTED;
    private double desiredExtension = Constants.intakeWristDefensivePositionRotations;

    private Slot0Configs slot0Configs = new Slot0Configs().withKP(0.2).withKI(0.003).withKD(0.0)
            .withKV(2.5).withKS(0.44); // DO NOT SET KD!!

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
        if (this.motor != null) {
            if (this.encoder == null){
                // wrist starts retracted
                this.motor.setPosition(Constants.intakeWristStartingPositionRotations);
            } else {
                this.motor.setPosition(this.encoder.getPosition().getValueAsDouble());
            }
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
        if (encoder != null) {
            return encoder.getPosition().getValueAsDouble();
        }
        if (motor != null) {
            return motor.getPosition().getValueAsDouble();
        }
        return 0.0;
    }

    /**
     * this method will be called once per scheduler run
     */
    @Override
    public void periodic() {
        if (motor == null) {
            if (encoder != null) {
                SmartDashboard.putNumber("IntakeWrist Encoder Position", encoder.getPosition().getValueAsDouble());
            }
            return;
        }
        if (!isManualMode) {
            if (!intakeWristIsAtDesiredExtension) {
                motor.setControl(voltageController.withPosition(desiredExtension));
                intakeWristIsAtDesiredExtension = isPositionWithinTolerance();
            } else {
                stop();
            }
        }
        SmartDashboard.putNumber("IntakeWrist Motor Encoder Position", getPosition());
        SmartDashboard.putBoolean("IntakeWrist Motor Output Velocity", motor.getMotionMagicAtTarget().getValue());
    }

    /**
     * A method to set the wrist position
     * @param position
     */
    public void setPosition(double position) {
        isManualMode = false;
        if (motor == null) {
            return;
        }
        desiredExtension = MathUtil.clamp(position, Constants.intakeWristDeployedPositionRotations,
                Constants.intakeWristDefensivePositionRotations);
        intakeWristIsAtDesiredExtension = false;
    }

    public void setMode(IntakeWristMode mode){
        isManualMode = false;
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
                    setPosition(Constants.intakeWristDefensivePositionRotations);
                    break;
                default:
                    setPosition(Constants.intakeWristDefensivePositionRotations);
            }
        }
    }

    // For manual control, auto sets the mode to manual
    public void runVoltage(double volts) {
        isManualMode = true;
        if (motor != null) {
            motor.setControl(voltageOut.withOutput(volts));
        }
    }

    public void stop() {
        if (motor != null) {
            motor.stopMotor();
        }
        intakeWristIsAtDesiredExtension = true;
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
            // when using the external sensor, don't configure the gear ratio here. 
            // use a 1:1 ratio and use positions directly from the encoder.  
            config.Feedback.SensorToMechanismRatio = 1.0;
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
        config.MotionMagic.MotionMagicCruiseVelocity = 50.0;
        config.MotionMagic.MotionMagicAcceleration = 25;
        config.MotionMagic.MotionMagicJerk = 50;

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
    boolean velocityIsSmall = motor == null
        || Math.abs(motor.getVelocity().getValueAsDouble()) < intakeWristLowVelocityTol;
        return positionTargetReached && velocityIsSmall;
    }
}
