// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ClimberPositionCommand.java
// Intent: command to set climber position
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPositionCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final DoubleSupplier targetHeightSupplier;

    /**
     * Creates a new ClimberPositionCommand.
     * @param climberSubsystem     The subsystem used by this command.
     * @param targetHeightSupplier A supplier that provides the target height in inches.
     */
    public ClimberPositionCommand(ClimberSubsystem climberSubsystem, DoubleSupplier targetHeightSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.targetHeightSupplier = targetHeightSupplier;

        addRequirements(climberSubsystem);
    }

    /**
     * Continuously updates the climber's setpoint to the value provided by the supplier.
     */
    @Override
    public void execute() {
        this.climberSubsystem.setManualMode(false);
        // if we have not zeroed, we should move up until we do, so we can get to a
        // known position. If we have zeroed, we can go to the target position.
        if (this.climberSubsystem.hasZeroed()) {
            this.climberSubsystem.goToPosition(targetHeightSupplier.getAsDouble());
            //this.climberSubsystem.stop();
        } else {
            this.climberSubsystem.runVoltage(3.0);
        }


    }

    /**
     * Returns true when the climber's current position and velocity are within 
     * the defined tolerances of the target.
     * @return True if the climber has reached the target position.
     */
    @Override
    public boolean isFinished() {
        return this.climberSubsystem.isClimberWithinTolerance();
    }
}