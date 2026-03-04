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

    private TalonFX spindexerTalonFX;
    private TofSensorLaser spindexerToF;

    private final VelocityVoltage motorController = new VelocityVoltage(0.0);

    private double targetRPS = 0.0;
    private boolean continuousMode = false;
    // Found based on experimentation on BearBones kicker V1
    private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.1199563795).withKV(0.1090512541).withKP(0.4)
            .withKD(0.0);

    /**
     * Constructor for SpindexerSpinner. Takes in a boolean staticMode which
     * determines whether the spindexer should run at a constant speed regardless of
     * the sensor state (true) or if it should stop when the sensor detects a ball
     * (false).
     * 
     * @param staticMode
     */
    public SpindexerSpinner(int motorCanID, int sensorCanID) {
        spindexerTalonFX = new TalonFX(motorCanID);
        spindexerToF = new TofSensorLaser(sensorCanID);
        configureMotor();
    }

    /*
     * Sets the target rpm
     */
    public void runRPMContinus() {
        this.targetRPS = rpmToRPS(rpmToRPS(Constants.spindexerSpeedRotationsPerMinute));
        this.continuousMode = true;
    }

    public void runRPMWtihSensor() {
        this.targetRPS = rpmToRPS(rpmToRPS(Constants.spindexerSpeedRotationsPerMinute));
        this.continuousMode = false;
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
        if (targetRPS == 0.0) {
            stop();
        } else if (!continuousMode && spindexerToF.tofActivated()) {
            stop();
        } else {
            motorController.withVelocity(targetRPS * Constants.spindexerGearRatio);
            spindexerTalonFX.setControl(motorController);
        }

        SmartDashboard.putNumber("Spindexer Real RPM", getRPM());
        spindexerToF.publishTelemetery();
    }

    /*
     * configures motor
     */
    private void configureMotor() {
        // from ElevatorSubsystem.java Reefscape2025
        // Config motor
        TalonFXConfiguration talonMotorConfig = new TalonFXConfiguration();
        talonMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
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
