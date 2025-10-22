// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: UpdateMotorSpeedCommand.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.ToFSensor;
import frc.robot.subsystems.NeoSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that updates a motor's speed based on a time-of-flight sensor
 * reading.
 * The computed speed is clamped between 0 and 1 and scaled before being applied
 * to the motor subsystem.
 */
public class UpdateMotorSpeedCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    /**
     * Motor subsystem controlled by this command.
     */
    private final NeoSubsystem motorSubsystem;

    /**
     * Time-of-flight sensor used to read distance (in inches).
     */
    private final ToFSensor tofSensor;

    /**
     * Create a new UpdateMotorSpeedCommand.
     *
     * @param motorSubsystem the subsystem that controls the motor
     * @param tofSensor      the ToF sensor providing range in inches
     */
    public UpdateMotorSpeedCommand(NeoSubsystem motorSubsystem, ToFSensor tofSensor) {
        // Store references to the subsystem and sensor.
        this.motorSubsystem = motorSubsystem;
        this.tofSensor = tofSensor;

        // Declare subsystem dependencies for the scheduler.
        addRequirements(motorSubsystem);
    }

    /**
     * Periodically called by the scheduler: reads the ToF sensor, computes a speed,
     * and sets the motor speed accordingly.
     */
    @Override
    public void execute() {
        SmartDashboard.putNumber("ToF Range", this.tofSensor.getRangeInches());
        double speed = Math.max(Math.min(1 - this.tofSensor.getRangeInches() / 7, 1), 0);
        this.motorSubsystem.setSpeed(speed * (3.0 / 4.0));
    }

    /**
     * Called when the command ends or is interrupted. Stops the motor.
     *
     * @param interrupted true if the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        this.motorSubsystem.stop();
    }

    /**
     * Called when the command is initially scheduled. Ensures the motor is stopped
     * on start.
     */
    @Override
    public void initialize() {
        // Ensure motor is stopped on start.
        this.motorSubsystem.stop();
    }

    /**
     * This command is designed to run indefinitely until canceled.
     *
     * @return false always
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
