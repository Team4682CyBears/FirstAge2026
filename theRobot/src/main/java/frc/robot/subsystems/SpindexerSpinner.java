// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: KickerSubsystem.java
// Intent: Run two krakens to kick the ball into the shooter
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.common.TofSensorLaser;

/* 
 * Runs the kicker which kicks the ball into the shooter consistiently
 */
/*
 * 
 * a) a run method that runs the spindexer at a constant rate indepedent of the sensor state. 
b) a run method that when set to run, if there is no ball in the kicker, the spindexer should run at the constant speed. If there is a ball in the kicker, the spindexer should stop. 
 */
public class SpindexerSpinner extends SubsystemBase {

    private TalonFX spindexerTalonFX = new TalonFX(Constants.spindexerSensorLaserCanID);
    //kickerTalonFX
    private TofSensorLaser spindexerToF = new TofSensorLaser(Constants.spindexerSensorLaserCanID);

    private final VelocityVoltage motorController = new VelocityVoltage(0.0);

    private double targetRPS = Constants.spindexerSpeed;
    private boolean staticMode = false;
    // Found based on experimentation on BearBones kicker V1
    private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.1199563795).withKV(0.1090512541).withKP(0.4)
            .withKD(0.0);

    /*
     * Initialize the kicker and configure the motor
     */
    public void initializeKickerSubsystem() {
        configureMotor();
    }

    /**
     * Constructor for SpindexerSpinner. Takes in a boolean staticMode which determines whether the spindexer should run at a constant speed regardless of the sensor state (true) or if it should stop when the sensor detects a ball (false).
     * @param staticMode
     */
    public SpindexerSpinner(boolean staticMode) {
        this.staticMode = staticMode;
        initializeKickerSubsystem();
    }
  
    /*
     * Sets the target rpm
     */
    public void runRPMStatic(double rpm) {
        this.targetRPS = rpmToRPS(rpm);
    }
    
    public void runRPM(double rpm) {
        if (spindexerToF.tofActivated()){
            stop();
        } else {
            runRPMStatic(rpm);
        }
        
    }


    private double rpmToRPS(double rpm) {
        return rpm / 60.0;
    }

    /*
     * Get the rpm from the lead motor
     */
    public double getRPM() {
        return spindexerTalonFX.getVelocity().getValueAsDouble() * 60 / Constants.kickerMotorGearRatio;
    }

    /*
     * Stop both motors and set the targetRPS to 0
     */
    public void stop() {
        targetRPS = 0.0;
        spindexerTalonFX.stopMotor();
    }

    /*
     * Run the motors every 20ms and log the real rpm
     */
    @Override
    public void periodic() {
        if (staticMode) { 
          runRPMStatic(targetRPS);
        } else {
          runRPM(targetRPS);
        }
        motorController.withVelocity(targetRPS * Constants.kickerMotorGearRatio);
        spindexerTalonFX.setControl(motorController);
        SmartDashboard.putNumber("Kicker Real RPM", getRPM());
        spindexerToF.publishTelemetery();
    }


    /*
     * configures motor
     */
    private void configureMotor() {
        // from ElevatorSubsystem.java Reefscape2025
        // Config motor
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

        StatusCode response = spindexerTalonFX.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + spindexerTalonFX.getDeviceID() + " failed config with error "
                            + response.toString());
        }

    }
}
