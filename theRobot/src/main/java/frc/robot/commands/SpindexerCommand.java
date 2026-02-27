

package frc.robot.commands;
import frc.robot.control.Constants;
import frc.robot.subsystems.SpindexerSpinner;
import edu.wpi.first.wpilibj2.command.Command;


public class SpindexerCommand  extends Command {
    private final SpindexerSpinner spindexerSpinner;

    public SpindexerCommand(SpindexerSpinner spindexerSpinner) {
        this.spindexerSpinner = spindexerSpinner;
        addRequirements(spindexerSpinner);
    }

    @Override
    public void execute() {
        spindexerSpinner.initializeKickerSubsystem();
        spindexerSpinner.runRPM(Constants.spindexerSpeed);
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
