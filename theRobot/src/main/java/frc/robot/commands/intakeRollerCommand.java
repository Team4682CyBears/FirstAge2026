// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: KickerCommand.java
// Intent: command to set kicker speed
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;

/**
 * Command to set the kicker speed.
 */
public class intakeRollerCommand extends Command {
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final DoubleSupplier intakeSpeedSupplier;

    /**
     * Constructs a new KickerCommand.
     * @param intakeRollerSubsystem
     * @param intakeSpeedSupplier
     */
    public intakeRollerCommand(IntakeRollerSubsystem intakeRollerSubsystem, DoubleSupplier intakeSpeedSupplier) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.intakeSpeedSupplier = intakeSpeedSupplier;
        addRequirements(intakeRollerSubsystem);
    }

    @Override
    public void initialize() {
        this.intakeRollerSubsystem.runRPM(intakeSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeRollerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until interrupted.
        return false;
    }
}
