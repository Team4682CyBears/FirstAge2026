// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: DefaultUpdateMotorSpeedCommand.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.ToFSensor;
import frc.robot.subsystems.NeoSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that updates a motor's speed based on a time-of-flight sensor
 * reading.
 * The computed speed is clamped between 0 and 1 and scaled before being applied
 * to the motor subsystem.
 */
public class DefaultUpdateMotorSpeedCommand extends Command {
    /**
     * Motor subsystem controlled by this command.
     */
    private final NeoSubsystem motorSubsystem;

    /**
     * Time-of-flight sensor used to read distance (in inches).
     */
    private final ToFSensor tofSensor;

    /**
     * Maximum range in inches that the motor will run. (this was randomly chosen after trial and error)
     */

    private static final double maxDetectionRange = 7.0;

    /**
     * Multiplies the speed by this before sending it to the motor. (this was randomly chosen after trial and error)
     */

    private static final double speedDeratingFactor = 0.75;

    /**
     * Create a new UpdateMotorSpeedCommand.
     *
     * @param motorSubsystem the subsystem that controls the motor
     * @param tofSensor      the ToF sensor providing range in inches
     */
    public DefaultUpdateMotorSpeedCommand(NeoSubsystem motorSubsystem, ToFSensor tofSensor) {
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
        // If the range is not valid then set the speed to 0
        if (tofSensor.isRangeValid()) {
            // 0 inches is when the speed is the fastest, `maxDetectionRange` inches or higher is when it is
            // the slowest. This is why the clamping is needed
            double speed = MathUtil.clamp(1 - tofSensor.getRangeInches() / maxDetectionRange, 0, 1);
            // `speedDeratingFactor` is used to scale the speed down because 1 was way
            // too fast for the test board
            motorSubsystem.setSpeed(speed * speedDeratingFactor);
        } else {
            motorSubsystem.setSpeed(0.0);
        }
    }

    /**
     * Called when the command ends or is interrupted. Stops the motor.
     *
     * @param interrupted true if the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        motorSubsystem.stop();
    }

    /**
     * Called when the command is initially scheduled. Ensures the motor is stopped
     * on start.
     */
    @Override
    public void initialize() {
        // Ensure motor is stopped on start.
        motorSubsystem.stop();
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
