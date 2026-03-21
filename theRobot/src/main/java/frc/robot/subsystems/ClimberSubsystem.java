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
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.HardwareConstants;

public class ClimberSubsystem extends SubsystemBase {
    private final SparkMax LeadMotor;
    private final SparkMax FollowMotor;
    private final DigitalInput hallEffectSensor;
    private boolean lastHallEffectState = false;

    private final SparkClosedLoopController PIDController;

    private static final double ROTATIONS_PER_INCH = ((25.0 * (34.0 / 24.0)) / 4) / 1.26504553; // gear ratio * calculated constant
    // position of the top of the sensor range
    private static final double SENSOR_POSITION_ABOVE_FLOOR_INCHES = 22.5; // Mostly Placeholder
    private static final double POSITION_TOLERANCE_INCHES = 0.25;
    private static final double VELOCITY_TOLERANCE_INCHES = 0.1; //inches per second
    private boolean hasZeroedYet = false;
    private boolean isManualMode = false;

    private double targetPositionInches = 0.0; 

    public static final double MIN_HEIGHT_ABOVE_FLOOR_INCHES = 21; // Placeholder
    public static final double MAX_HEIGHT_INCHES = 30.0; // Placeholder

    /**
     * Initializes the climber hardware and configuration.
     * @param LeadCanID   The CAN ID for the leader Spark Flex.
     * @param FollowCanID The CAN ID for the follower Spark Flex.
     * @param DIOPortID   The RoboRIO DIO port for the Hall Effect sensor.
     */
    public ClimberSubsystem(int LeadCanID, int FollowCanID, int DIOPortID) {
        this.LeadMotor = new SparkMax(LeadCanID, MotorType.kBrushless);
        this.FollowMotor = new SparkMax(FollowCanID, MotorType.kBrushless);
        this.PIDController = this.LeadMotor.getClosedLoopController();
        this.hallEffectSensor = new DigitalInput(DIOPortID);

        SmartDashboard.putNumber("Go to climber position", 28.0);
        LeadMotor.getEncoder().setPosition(MIN_HEIGHT_ABOVE_FLOOR_INCHES);

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
        double clampedPosition = MathUtil.clamp(targetInches, MIN_HEIGHT_ABOVE_FLOOR_INCHES, MAX_HEIGHT_INCHES);
        targetPositionInches = clampedPosition;
        PIDController.setSetpoint(clampedPosition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }
    

    /**
     * Commands the climber to move to a specific height. The input is clamped 
     * between the minimum and maximum physical limits.
     * @param targetInches The target height in inches relative to the floor.
     */
    public void runVelocity(double inchesPerSecond) {
        PIDController.setSetpoint(inchesPerSecond, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    /**
     * Determines if the climber is near its target height and has stopped moving.
     * @return True if within both position and velocity tolerances.
     */
    public boolean isClimberWithinTolerance() {
        return (Math.abs(getPosition() - targetPositionInches) < POSITION_TOLERANCE_INCHES) 
            && (Math.abs(getVelocity()) < VELOCITY_TOLERANCE_INCHES);
    }

    /**
     * Standard subsystem periodic loop. Handles telemetry and automatic 
     * encoder recalibration when the Hall Effect sensor transition is detected.
     */
    @Override
    public void periodic() {
        boolean currentDetected = isDetected();
        double velocity = getVelocity();

        SmartDashboard.putBoolean("Climber Sensor Detected", currentDetected);

        SmartDashboard.putNumber("Climber Position Inches", getPosition());
        SmartDashboard.putNumber("Climber Velocity Inches Per Second", getVelocity());

        // if we are going up and we previously detected it and now we don't, zero the sensor
        if ((!this.hasZeroedYet || this.isManualMode) && (lastHallEffectState && !currentDetected)) {
            LeadMotor.getEncoder().setPosition(SENSOR_POSITION_ABOVE_FLOOR_INCHES);
            hasZeroedYet = true;
        }
        // if we are going down and we previously didn't detected it and now we do, zero the sensor
        else if ((!this.hasZeroedYet || this.isManualMode) && (!lastHallEffectState && currentDetected)) {
            LeadMotor.getEncoder().setPosition(SENSOR_POSITION_ABOVE_FLOOR_INCHES);
            hasZeroedYet = true;
        }
        lastHallEffectState = currentDetected;
    }

    public boolean hasZeroed() {
        return hasZeroedYet;
    }

    public void setManualMode(boolean manualMode) {
        this.isManualMode = manualMode;
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
            System.out.println("SparkMax ID " + id + " failed config: " + error);
        }
    }

    private void configureMotors() {
        SparkMaxConfig leadConfig = new SparkMaxConfig();
        double positionConversion = 1.0 / ROTATIONS_PER_INCH;
        
        leadConfig.idleMode(IdleMode.kBrake); 
        leadConfig.smartCurrentLimit(HardwareConstants.shooterSmartCurrentLimitAmps);
        leadConfig.voltageCompensation(HardwareConstants.nominalVoltageCompensationVolts);
        leadConfig.inverted(true);


        leadConfig.encoder
            .positionConversionFactor(positionConversion)
            .velocityConversionFactor(positionConversion / 60.0);

        leadConfig.closedLoop
            .p(0.2)
            .i(0.0)
            .d(0.0)
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .apply(new FeedForwardConfig().kS(.15).kV(.002));

        REVLibError error = LeadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        checkError(error, LeadMotor.getDeviceId());

        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.idleMode(IdleMode.kBrake);
        followConfig.follow(LeadMotor);

        error = FollowMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        checkError(error, FollowMotor.getDeviceId());
    }

    private boolean isDetected() {
        return !hallEffectSensor.get();
    }
}