// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: KickerSpindexerCommand.java
// Intent: command to run kicker and spindexer together
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.control.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexerSpinner;

/**
 * Command to run the kicker and spindexer together.
 */
public class KickerSpindexerCommand extends Command {
    private final KickerSubsystem kickerSubsystem;
    private final SpindexerSpinner spindexerSpinner;

    /**
     * Constructs a new KickerSpindexerCommand.
     *
     * @param kickerSubsystem
     * @param spindexerSpinner
     * @param kickSpeedSupplier
     */
    public KickerSpindexerCommand(
            KickerSubsystem kickerSubsystem,
            SpindexerSpinner spindexerSpinner) {
        this.kickerSubsystem = kickerSubsystem;
        this.spindexerSpinner = spindexerSpinner;
        addRequirements(kickerSubsystem, spindexerSpinner);
    }

    @Override
    public void initialize() {
        kickerSubsystem.runRPM(Constants.KICKER_RPM);
        spindexerSpinner.runRPMContinus();
    }

    @Override
    public void end(boolean interrupted) {
        kickerSubsystem.stop();
        spindexerSpinner.stop();
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until interrupted.
        return false;
    }
}
