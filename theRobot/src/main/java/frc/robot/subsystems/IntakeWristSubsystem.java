package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.control.IntakeWristMode;

public class IntakeWristSubsystem extends SubsystemBase {

    // wrist gearing
    private static final double intakeWristGearRatio = 1.0/5.0 * 1.0/5.0 * 22.0/32.0;
    private static final double intakeWristLowVelocityTol = 10; // TODO test this on device with motion magic profile

    private TalonFX motor;
    private MotionMagicVoltage voltageController = new MotionMagicVoltage(0.0);

    private boolean intakeWristIsAtDesiredExtension = true;
    private IntakeWristMode intakeWristMode = IntakeWristMode.RETRACTED;
    private double desiredExtension = Constants.intakeWristRetractedPositionRotations;

    private Slot0Configs slot0Configs = new Slot0Configs().withKP(0.5).withKI(0.003).withKD(0.0).withKG(0.22)
            .withKV(3.85).withKS(0.5); // TODO: Find real values. DO NOT SET KD!!

    public IntakeWristSubsystem(int motorCanID) {
        if (InstalledHardware.intakeWristMotorInstalled) {
            this.motor = new TalonFX(motorCanID);
            configureMotor();
        }
        this.motor.setPosition(Constants.intakeWristRetractedPositionRotations);
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
        intakeWristIsAtDesiredExtension = isExtendoWithinTolerance();
        if (!intakeWristIsAtDesiredExtension){
            motor.setControl(voltageController.withPosition(desiredExtension));
        }
        SmartDashboard.putNumber("IntakeWrist Motor Encoder Extendo", getPosition());
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

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        config.Slot0 = slot0Configs;
        config.ClosedLoopRamps.withVoltageClosedLoopRampPeriod(0.02);

        config.Feedback.SensorToMechanismRatio = 1.0/intakeWristGearRatio;

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
     * A method to test whether the extendo is within tolerance of the target
     * extendo
     * 
     * @return true if the extendo is within tolerance
     */
    private boolean isExtendoWithinTolerance() {
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
