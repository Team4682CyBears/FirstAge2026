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

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;

/* 
 * Runs the kicker which kicks the ball into the shooter consistiently
 */
public class KickerSubsystem extends SubsystemBase {

    // TODO: Remove the follower in the final mechanism
    private TalonFX kickerLeadTalonFX = new TalonFX(Constants.kickerLeadTalonCanId);
    private TalonFX kickerFollowTalonFX = new TalonFX(Constants.kickerFollowTalonCanId);

    private final VelocityVoltage leaderController = new VelocityVoltage(0.0);
    private final VelocityVoltage followerController = new VelocityVoltage(0.0);

    private double targetRPS = 0.0;

    private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.09009009009).withKV(0.4504504505).withKP(0.4)
            .withKD(0.0);

    /*
     * Initialize the kicker and configure the motor
     */
    public KickerSubsystem() {
        configureMotor();
    }

    /*
     * Sets the target rpm
     */
    public void runRPM(double rpm) {
        this.targetRPS = rpm / 60.0;
        leaderController.withVelocity(targetRPS);
        followerController.withVelocity(targetRPS * Constants.followKickerMotorGearRatio);
        kickerLeadTalonFX.setControl(leaderController);
        kickerFollowTalonFX.setControl(followerController);
    }

    /*
     * Get the rpm from the lead motor
     */
    public double getRPM() {
        return kickerLeadTalonFX.getVelocity().getValueAsDouble() * 60;
    }

    /*
     * Stop both motors and set the targetRPS to 0
     */
    public void stop() {
        targetRPS = 0.0;
        kickerLeadTalonFX.stopMotor();
        kickerFollowTalonFX.stopMotor();
    }

    /*
     * Run the motors every 20ms and log the real rpm
     */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Kicker Real RPM", getRPM());
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
        talonMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode response = kickerLeadTalonFX.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + kickerLeadTalonFX.getDeviceID() + " failed config with error "
                            + response.toString());
        }

        // change invert for angleRightMotor
        talonMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        // apply configs
        response = kickerFollowTalonFX.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            DataLogManager.log(
                    "TalonFX ID " + kickerFollowTalonFX.getDeviceID() + " failed config with error "
                            + response.toString());
        }

    }
}
