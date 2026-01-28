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

public class ShooterSubsystem extends SubsystemBase {
    private final SparkFlex LeadMotor;
    private final SparkFlex FollowMotor;

    private final SparkClosedLoopController PIDController;

    private final DoubleLogEntry speedLogEntry;

    // max rpm 6784.0
    public ShooterSubsystem(int LeadCanID, int FollowCanID) {
        this.LeadMotor = new SparkFlex(LeadCanID, MotorType.kBrushless);
        this.FollowMotor = new SparkFlex(FollowCanID, MotorType.kBrushless);
        this.speedLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/shooter/rpm");
        PIDController = LeadMotor.getClosedLoopController();
        SparkFlexConfig LeadConfig = new SparkFlexConfig();

        LeadConfig.idleMode(IdleMode.kCoast);
        LeadConfig.smartCurrentLimit(40);
        LeadConfig.voltageCompensation(12.0);

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

    public void runVoltage(double Volts) {
        LeadMotor.setVoltage(Volts);
    }

    public void runRPM(double targetRPM) {
        PIDController.setSetpoint(targetRPM, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
    }

    public double getRPM() {
        try {
            return LeadMotor.getEncoder().getVelocity();
        } catch (Exception e) {
            return -1;
        }
    }

    public void stop() {
        LeadMotor.stopMotor();
    }

    @Override
    public void periodic() {
        speedLogEntry.append(getRPM());
    }
}
