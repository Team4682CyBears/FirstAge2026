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
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
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

    private final DoubleLogEntry speedLogEntry;

    /*
     * Initialize and configure the shooter motors and PIDs
     */
    public ShooterSubsystem(int LeadCanID, int FollowCanID) {
        this.LeadMotor = new SparkFlex(LeadCanID, MotorType.kBrushless);
        this.FollowMotor = new SparkFlex(FollowCanID, MotorType.kBrushless);
        this.speedLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/shooter/rpm");
        PIDController = LeadMotor.getClosedLoopController();
        SparkFlexConfig LeadConfig = new SparkFlexConfig();

        LeadConfig.idleMode(IdleMode.kCoast);
        LeadConfig.smartCurrentLimit(HardwareConstants.shooterSmartCurrentLimitAmps);
        LeadConfig.voltageCompensation(HardwareConstants.nominalVoltageCompensationVolts);

        LeadConfig.closedLoop
                .p(.00027)
                .i(0.0)
                .d(0.001)
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .apply(new FeedForwardConfig().kS(.15).kV(.002)); // TODO: Calculate values for kS and kV

        LeadMotor.configure(LeadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkFlexConfig FollowerConfig = new SparkFlexConfig();

        FollowerConfig.idleMode(IdleMode.kCoast);
        FollowerConfig.smartCurrentLimit(40);
        FollowerConfig.follow(LeadMotor, true);

        FollowMotor.configure(FollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        try {
            return LeadMotor.getEncoder().getVelocity();
        } catch (Exception e) {
            return -1;
        }
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
        SmartDashboard.putNumber("Shooter RPM", rpm);
        speedLogEntry.append(rpm);
    }
}
