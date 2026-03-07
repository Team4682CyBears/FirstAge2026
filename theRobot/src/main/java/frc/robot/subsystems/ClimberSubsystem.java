// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ClimberSubsystem.java
// Intent: Forms the prelminary code for the climber subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.HardwareConstants;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;



public class ClimberSubsystem extends SubsystemBase {
    private final SparkFlex LeadMotor;
    private final SparkFlex FollowMotor;
    private final DigitalInput hallEffectSensor;
    private boolean lastHallEffectState = false;

    private final SparkClosedLoopController PIDController;
    /*
     * TODO: Get mech to get precise values
     */
    private static final double ROTATIONS_PER_INCH = 2.0; // Placeholder
    private static final double SENSOR_POSITION_INCHES = 1.0; // Mostly Placeholder
    private static final double MIN_HEIGHT_INCHES = 0.0; // Placeholder
    private static final double MAX_HEIGHT_INCHES = 25.0; // Placeholder
    private static final double POSITION_TOLERANCE = 0.5; // unsure of what exactly tolerance should be, couldn't find anything on tolerance in KickerSubsystem
    private static final double VELOCITY_TOLERANCE = 0.1; // that goes for method too, both values placeholders

    /*
     * Initialize and configure the shooter motors and PIDs
     */
    public ClimberSubsystem(int LeadCanID, int FollowCanID) {
        this.LeadMotor = new SparkFlex(LeadCanID, MotorType.kBrushless);
        this.FollowMotor = new SparkFlex(FollowCanID, MotorType.kBrushless);
        this.PIDController = this.LeadMotor.getClosedLoopController();
        this.hallEffectSensor = new DigitalInput(0);

        configureMotors();
    }


    /**
     * Checks if the climber is at the target and moving slowly enough to be considered "set".
     */
    public boolean isClimberWithinTolerance(double targetInches) {
        return (Math.abs(getPosition() - targetInches) < POSITION_TOLERANCE) 
            && (Math.abs(getVelocity()) < VELOCITY_TOLERANCE);
    }

    public void goToPosition(double targetInches) {
        double clampedPosition = MathUtil.clamp(targetInches, MIN_HEIGHT_INCHES, MAX_HEIGHT_INCHES);
        PIDController.setSetpoint(clampedPosition, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    }

     /*
      * Run the climber at the target voltage
      */
    public void runVoltage(double Volts) {
        LeadMotor.setVoltage(Volts);
    }
    public void stop() {
        LeadMotor.stopMotor();
    }

    public double getPosition() {
        return LeadMotor.getEncoder().getPosition();
    }

    public double getVelocity() {
        return LeadMotor.getEncoder().getVelocity();
    }

    /*
     * determines if the climber should reset its encoder based on the hall effect sensor
     * logic: if the climber is moving up and the sensor was previously triggered but is now not triggered,
     * or if the climber is moving down and the sensor was previously not triggered but is now triggered, then we should reset the encoder
     */
    @Override
    public void periodic() {
        boolean currentDetected = isDetected();
        double velocity = getVelocity(); // Assumption: positive velocity means moving up, negative velocity means moving down

        // 1. Check for Reset Conditions
        // Upwards: Detected -> Not Detected
        if (velocity > 0.1 && (lastHallEffectState && !currentDetected)) {
            LeadMotor.getEncoder().setPosition(SENSOR_POSITION_INCHES);
        } 
        // Downwards: Not Detected -> Detected
        else if (velocity < -0.1 && (!lastHallEffectState && currentDetected)) {
            LeadMotor.getEncoder().setPosition(SENSOR_POSITION_INCHES);
        }
        lastHallEffectState = currentDetected;
    }
    private void checkError(REVLibError error, int id) {
        if (error != REVLibError.kOk) {
            System.out.println("SparkFlex ID " + id + " failed config: " + error);
        }
    }
    
    private boolean isDetected() {
        return !hallEffectSensor.get();
    }
    
    private void configureMotors() {
        SparkFlexConfig leadConfig = new SparkFlexConfig();

        // Conversion factor: 1 / rotationsPerInch converts rotations to inches
        double positionConversion = 1.0 / ROTATIONS_PER_INCH;
        
        leadConfig.idleMode(IdleMode.kBrake); 
        leadConfig.smartCurrentLimit(HardwareConstants.shooterSmartCurrentLimitAmps);
        leadConfig.voltageCompensation(HardwareConstants.nominalVoltageCompensationVolts);

        // Configure Encoder units
        leadConfig.encoder
            .positionConversionFactor(positionConversion)
            .velocityConversionFactor(positionConversion / 60.0); // inches per second

        leadConfig.closedLoop
            .p(0.1) // Start small for position control
            .i(0.0)
            .d(0.0);

        REVLibError error = LeadMotor.configure(leadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        checkError(error, LeadMotor.getDeviceId());

        SparkFlexConfig followConfig = new SparkFlexConfig();
        followConfig.idleMode(IdleMode.kBrake);
        followConfig.follow(LeadMotor); 

        error = FollowMotor.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        checkError(error, FollowMotor.getDeviceId());
    }
}
