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
import frc.robot.subsystems.KickerSubsystem;

public class KickerCommand extends Command {
    private final KickerSubsystem kickerSubsystem;
    private final DoubleSupplier kickSpeedSupplier;

    public KickerCommand(KickerSubsystem kickerSubsystem, DoubleSupplier kickSpeedSupplier) {
        this.kickerSubsystem = kickerSubsystem;
        this.kickSpeedSupplier = kickSpeedSupplier;
        addRequirements(kickerSubsystem);
    }

    @Override
    public void initialize() {
        this.kickerSubsystem.runRPM(kickSpeedSupplier.getAsDouble());
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
