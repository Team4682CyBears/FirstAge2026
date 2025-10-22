// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: NeoSubsystem.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem wrapper for a REV Neo motor controller.
 */
public class NeoSubsystem extends SubsystemBase {

    SparkMax neoMotor;

    /** Creates a new NeoSubsystem. */
    /**
     * Create a new NeoSubsystem for the given CAN ID.
     *
     * @param canID the CAN ID of the Neo motor controller
     */
    public NeoSubsystem(int canID) {
        this.neoMotor = new SparkMax(canID, MotorType.kBrushless);

        neoMotor.set(0.0);

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);

        neoMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    // Set the motor speed to the clamped speed
    /**
     * Set the motor speed.
     *
     * @param speed motor output in range [-1.0, 1.0]
     */
    public void setSpeed(double speed) {
        neoMotor.set(speed);
    }

    // Stop the motor
    /**
     * Stop the motor immediately.
     */
    public void stop() {
        neoMotor.stopMotor();
    }
}
