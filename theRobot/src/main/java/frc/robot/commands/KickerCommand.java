// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: KickerCommand.java
// Intent: command to set kicker speed
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.control.Constants;

/**
 * Command to set the kicker speed.
 */
public class KickerCommand extends Command {
    private final KickerSubsystem kickerSubsystem;

    /**
     * Constructs a new KickerCommand.
     * @param kickerSubsystem
     */
    public KickerCommand(KickerSubsystem kickerSubsystem) {
        this.kickerSubsystem = kickerSubsystem;
        addRequirements(kickerSubsystem);
    }

    @Override
    public void initialize() {
        this.kickerSubsystem.runRPM(Constants.KICKER_RPM);
    }

    @Override
    public void end(boolean interrupted) {
        this.kickerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until interrupted.
        return false;
    }
}
