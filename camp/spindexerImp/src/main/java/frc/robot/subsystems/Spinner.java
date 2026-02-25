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
import frc.robot.Constants;
import frc.robot.subsystems.Spinner;

/* 
 * Runs the spinner which spins the ball into the shooter consistently
 */
public class Spinner extends SubsystemBase {

    private TalonFX spinnerTalonFX;
    // todo make private lazer can 
    
    private final VelocityVoltage motorController = new VelocityVoltage(0.0);

    private double targetRPS = Constants.spindexerSpeed;
    private boolean useSensor = false;

    // Found based on experimentation on BearBones kicker V1
    private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.1199563795).withKV(0.1090512541).withKP(0.52)
            .withKD(0.0);

    /*
     * Initialize the kicker and configure the motor
     */
    public Spinner(int motorCanID, int sensorCanID) {
        spinnerTalonFX = new TalonFX(motorCanID);

        configureMotor();
    }

    /*
     * Sets the target rpm
     */
    public void runRPM(double rpm) {
        this.targetRPS = rpmToRPS(rpm);
        useSensor = false;
    }

      /*
     * Sets the target rpm
     */
    public void runRPMWithSensor(double rpm) {
        this.targetRPS = rpmToRPS(rpm);
        useSensor = true;
  
    }

    private double rpmToRPS(double rpm) {
        return rpm / 60.0;
    }

    /*
     * Get the rpm from the lead motor
     */
    public double getRPM() {
        return kickerTalonFX.getVelocity().getValueAsDouble() * 60 / Constants.kickerMotorGearRatio;
    }

    /*
     * Stop both motors and set the targetRPS to 0
     */
    public void stop() {
        targetRPS = 0.0;
        kickerTalonFX.stopMotor();
    }

    
    /*
     * Run the motors every 20ms and log the real rpm
     */
    @Override
    public void periodic() {
      //if sensor enabled...
      motorController.withVelocity(targetRPS * Constants.spinnerMotorGearRatio);
      spinnerTalonFX.setControl(motorController);
      SmartDashboard.putNumber("Spinner Real RPM", getRPM());
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

        StatusCode response = kickerTalonFX.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + kickerTalonFX.getDeviceID() + " failed config with error "
                            + response.toString());
        }

    }
}
