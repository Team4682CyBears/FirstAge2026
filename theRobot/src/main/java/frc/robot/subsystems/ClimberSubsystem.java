// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ClimberSubsystem.java
// Intent: Forms the prelminary code for the climber subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.HardwareConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkFlex LeadMotor;
    private final SparkFlex FollowMotor;
    private final DigitalInput hallEffectSensor;
    private boolean lastHallEffectState = false;

    private final SparkClosedLoopController PIDController;

    private static final double ROTATIONS_PER_INCH = 2.0; // Placeholder
    // position of the top of the sensor range
    private static final double SENSOR_POSITION_ABOVE_FLOOR_INCHES = 1.0; // Mostly Placeholder
    private static final double MIN_HEIGHT_ABOVE_FLOOR_INCHES = 0.0; // Placeholder
    private static final double MAX_HEIGHT_INCHES = 25.0; // Placeholder
    private static final double POSITION_TOLERANCE_INCHES = 0.25; 
    private static final double VELOCITY_TOLERANCE = 0.1; 

    private double targetPositionInches = 0.0; 

    /**
     * Initializes the climber hardware and configuration.
     * @param LeadCanID   The CAN ID for the leader Spark Flex.
     * @param FollowCanID The CAN ID for the follower Spark Flex.
     * @param DIOPortID   The RoboRIO DIO port for the Hall Effect sensor.
     */
    public ClimberSubsystem(int LeadCanID, int FollowCanID, int DIOPortID) {
        this.LeadMotor = new SparkFlex(LeadCanID, MotorType.kBrushless);
        this.FollowMotor = new SparkFlex(FollowCanID, MotorType.kBrushless);
        this.PIDController = this.LeadMotor.getClosedLoopController();
        this.hallEffectSensor = new DigitalInput(DIOPortID);

        SmartDashboard.putNumber("Go to climber position", 1.0);

        configureMotors();
    }

    /**
     * Returns the current height of the climber.
     * @return Current position in inches (calculated via conversion factor).
     */
    public double getPosition() {
        return LeadMotor.getEncoder().getPosition();
    }

    /**
     * Gets the currently set target height of the climber.
     * @return The target position in inches.
     */
    public double getTargetPosition() {
        return targetPositionInches;
    }

    /**
     * Returns the current speed of the climber.
     * @return Current velocity in inches per second.
     */
    public double getVelocity() {
        return LeadMotor.getEncoder().getVelocity();
    }

    /**
     * Commands the climber to move to a specific height. The input is clamped 
     * between the minimum and maximum physical limits.
     * @param targetInches The target height in inches relative to the floor.
     */
    public void goToPosition(double targetInches) {
        targetPositionInches = targetInches;
        double clampedPosition = MathUtil.clamp(targetInches, MIN_HEIGHT_ABOVE_FLOOR_INCHES, MAX_HEIGHT_INCHES);
        PIDController.setSetpoint(clampedPosition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

    /**
     * Determines if the climber is near its target height and has stopped moving.
     * @param targetInches The desired height in inches.
     * @return True if within both position and velocity tolerances.
     */
    public boolean isClimberWithinTolerance(double targetInches) {
        return (Math.abs(getPosition() - targetInches) < POSITION_TOLERANCE_INCHES) 
            && (Math.abs(getVelocity()) < VELOCITY_TOLERANCE);
    }

    /**
     * Standard subsystem periodic loop. Handles telemetry and automatic 
     * encoder recalibration when the Hall Effect sensor transition is detected.
     */
    @Override
    public void periodic() {
        boolean currentDetected = isDetected();
        double velocity = getVelocity();

        // if we are going up and we previously detected it and now we don't, zero the sensor
        if (velocity > 0.1 && (lastHallEffectState && !currentDetected)) {
            LeadMotor.getEncoder().setPosition(SENSOR_POSITION_ABOVE_FLOOR_INCHES);
        }
        // if we are going down and we previously didn't detected it and now we do, zero the sensor
        else if (velocity < -0.1 && (!lastHallEffectState && currentDetected)) {
            LeadMotor.getEncoder().setPosition(SENSOR_POSITION_ABOVE_FLOOR_INCHES);
        }
        lastHallEffectState = currentDetected;
    }

    /**
     * Drives the lead climber motor using a raw voltage.
     * @param Volts The voltage to apply (-12.0 to 12.0).
     */
    public void runVoltage(double Volts) {
        LeadMotor.setVoltage(Volts);
    }

    /**
     * Immediately stops motor movement and sets voltage to 0.
     */ 
    public void stop() {
        LeadMotor.stopMotor();
    }

    private void checkError(REVLibError error, int id) {
        if (error != REVLibError.kOk) {
            System.out.println("SparkFlex ID " + id + " failed config: " + error);
        }
    }

    private void configureMotors() {
        SparkFlexConfig leadConfig = new SparkFlexConfig();
        double positionConversion = 1.0 / ROTATIONS_PER_INCH;
        
        leadConfig.idleMode(IdleMode.kBrake); 
        leadConfig.smartCurrentLimit(HardwareConstants.shooterSmartCurrentLimitAmps);
        leadConfig.voltageCompensation(HardwareConstants.nominalVoltageCompensationVolts);

        leadConfig.encoder
            .positionConversionFactor(positionConversion)
            .velocityConversionFactor(positionConversion / 60.0);

        leadConfig.closedLoop
            .p(0.1)
            .i(0.0)
            .d(0.0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .apply(new FeedForwardConfig().kS(.15).kV(.002));

        REVLibError error = LeadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        checkError(error, LeadMotor.getDeviceId());

        SparkFlexConfig followConfig = new SparkFlexConfig();
        followConfig.idleMode(IdleMode.kBrake);
        followConfig.follow(LeadMotor); 
        followConfig.inverted(true);

        error = FollowMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        checkError(error, FollowMotor.getDeviceId());
    }

    private boolean isDetected() {
        return !hallEffectSensor.get();
    }
}