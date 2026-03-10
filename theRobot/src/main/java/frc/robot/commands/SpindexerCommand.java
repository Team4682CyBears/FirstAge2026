
// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: SpindexerCommand.java
// Intent: command to run the spindexer
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.SpindexerSpinner;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A method to run the spindexer
 */
public class SpindexerCommand extends Command {
    private final SpindexerSpinner spindexerSpinner;
    private final boolean continuousMode;

    /**
     * Constructor for SpindexerCommand
     * @param spindexerSpinner
     * @param continuousMode - true if it should run continuously, false if it should stop
     * when the sensor is activated
     */
    public SpindexerCommand(SpindexerSpinner spindexerSpinner, boolean continuousMode) {
        this.spindexerSpinner = spindexerSpinner;
        this.continuousMode = continuousMode;
        addRequirements(spindexerSpinner);
    }

    /**
     * Called when the command starts
     */
    @Override
    public void initialize() {
        if (continuousMode) {
            spindexerSpinner.runRPMContinus();
        } else {
            spindexerSpinner.runRPMWtihSensor();
        }
    }

    /**
     * Called when the command ends or is interrupted
     */
    @Override
    public void end(boolean interrupted) {
        spindexerSpinner.stop();
    }

    /**
     * determines when the command is finished
     */
    @Override
    public boolean isFinished() {
        // this command is intended to be run based on a trigger, so should 
        // always return false.
        return false;
    }

}
