// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: IntakeRoller.jave
// Intent: Moves balls into the intake
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

/* 
 * Runs the Intake which intakes the ball into the robot
 */
public class IntakeRollerSubsystem extends SubsystemBase {
    private TalonFX intakeTalonFX;
    private IntakeWristSubsystem wrist;
    private boolean overrideWristConstraint = false;
    private boolean shouldStop = true;
    private final VelocityVoltage leaderController = new VelocityVoltage(0.0);
    private double targetRPS = 0.0;

    private Slot0Configs slot0Configs = new Slot0Configs().withKS(0.09009009009).withKV(0.4504504505).withKP(0.4)
            .withKD(0.0);

    /*
     * Initialize the intake roller and configure the motor
     */
    public IntakeRollerSubsystem(int motorCanID, IntakeWristSubsystem wrist) {
        intakeTalonFX = new TalonFX(motorCanID);
        this.wrist = wrist;
        configureMotor();
    }

    /*
     * Get the rpm from the lead motor
     */
    public double getRPM() {
        return intakeTalonFX.getVelocity().getValueAsDouble() * 60;
    }

    /*
     * Run the motors every 20ms and log the real rpm
     */
    @Override
    public void periodic() {
        if ((wrist.getPosition() <= Constants.intakeWristAngleGoodToRoll || overrideWristConstraint) && !shouldStop){
            leaderController.withVelocity(targetRPS);
            intakeTalonFX.setControl(leaderController);
            SmartDashboard.putNumber("Intake Real RPM", getRPM());
        } else {
            intakeTalonFX.stopMotor();
        }
    }

    /*
     * Sets the target rpm
     */
    public void runRPM(double rpm) {
        shouldStop = false;
        this.targetRPS = rpmToRPS(rpm);
    }

    /*
     * Stop motor and set the targetRPS to 0
     */
    public void stop() {
        shouldStop = true;
        intakeTalonFX.stopMotor();
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
        talonMotorConfig.CurrentLimits.StatorCurrentLimit = 70.0;
        talonMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        talonMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        StatusCode response = intakeTalonFX.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + intakeTalonFX.getDeviceID() + " failed config with error "
                            + response.toString());
        }
    }

    private double rpmToRPS(double rpm) {
        return rpm / 60.0;
    }

    public void setOverrideWristConstraint(boolean shouldOverrideWristConstraint){
        this.overrideWristConstraint = shouldOverrideWristConstraint;
    }

    public boolean getOverrideWristConstraint(){
        return overrideWristConstraint;
    }
}
