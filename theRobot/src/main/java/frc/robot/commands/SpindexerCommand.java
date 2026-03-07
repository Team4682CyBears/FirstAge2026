
package frc.robot.commands;

import frc.robot.control.Constants;
import frc.robot.subsystems.SpindexerSpinner;
import edu.wpi.first.wpilibj2.command.Command;

public class SpindexerCommand extends Command {
    private final SpindexerSpinner spindexerSpinner;
    private final boolean continuousMode;

    public SpindexerCommand(SpindexerSpinner spindexerSpinner, boolean continuousMode) {
        this.spindexerSpinner = spindexerSpinner;
        this.continuousMode = continuousMode;
        addRequirements(spindexerSpinner);
    }

    @Override
    public void initialize() {
        if (continuousMode) {
            spindexerSpinner.runRPMContinus();
        } else {
            spindexerSpinner.runRPMWtihSensor();
        }
    }

    @Override
    public void end(boolean interrupted) {
        spindexerSpinner.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
