// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShooterSubsystem.java
// Intent: Use spark flex motors to shoot the ball
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.HardwareConstants;

/*
 * Shoots the ball using two mechanically connected spark flex motors
 */
public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex LeadMotor;
    private final SparkFlex FollowMotor;

    private final SparkClosedLoopController PIDController;

    /*
     * Initialize and configure the shooter motors and PIDs
     */
    public ShooterSubsystem(int LeadCanID, int FollowCanID) {
        this.LeadMotor = new SparkFlex(LeadCanID, MotorType.kBrushless);
        this.FollowMotor = new SparkFlex(FollowCanID, MotorType.kBrushless);
        this.PIDController = this.LeadMotor.getClosedLoopController();
        configureMotors();
    }

    /*
     * Run the shooter at the target voltage
     */
    public void runVoltage(double Volts) {
        LeadMotor.setVoltage(Volts);
    }

    /*
     * Run the motor at the target velocity in rpm
     */
    public void runRPM(double targetRPM) {
        PIDController.setSetpoint(targetRPM, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    /*
     * Get the rpm from the lead motor
     */
    public double getRPM() {
        return LeadMotor.getEncoder().getVelocity();
    }

    /*
     * stop the motors
     */
    public void stop() {
        LeadMotor.stopMotor();
    }

    /*
     * log the rpm every 20ms
     */
    @Override
    public void periodic() {
        double rpm = getRPM();
        SmartDashboard.putNumber("Real Shooter RPM", rpm);
    }

    private void configureMotors() {
        SparkFlexConfig LeadConfig = new SparkFlexConfig();

        LeadConfig.idleMode(IdleMode.kCoast);
        LeadConfig.smartCurrentLimit(HardwareConstants.shooterSmartCurrentLimitAmps);
        LeadConfig.voltageCompensation(HardwareConstants.nominalVoltageCompensationVolts);
        LeadConfig.inverted(true);

        // Derived values from testing on tardi
        LeadConfig.closedLoop
                .p(.00027)
                .i(0.0)
                .d(0.001)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .apply(new FeedForwardConfig().kS(.15).kV(.002)); // TODO: Calculate values for kS and kV

        REVLibError error = LeadMotor.configure(LeadConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        if (error != REVLibError.kOk) {
            System.out.println(
                    "SparkFlex ID " + LeadMotor.getDeviceId() + " failed config with error "
                            + error.toString());
        }

        SparkFlexConfig FollowerConfig = new SparkFlexConfig();

        FollowerConfig.idleMode(IdleMode.kCoast);
        FollowerConfig.smartCurrentLimit(HardwareConstants.shooterSmartCurrentLimitAmps);
        FollowerConfig.follow(LeadMotor, true); // invert the follower motors

        error = FollowMotor.configure(FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        if (error != REVLibError.kOk) {
            System.out.println(
                    "SparkFlex ID " + FollowMotor.getDeviceId() + " failed config with error "
                            + error.toString());
        }
    }
}
