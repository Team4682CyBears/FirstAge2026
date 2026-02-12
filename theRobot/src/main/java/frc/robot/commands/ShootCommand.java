// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShootCommand.java
// Intent: command to set shooter speed
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * Command to set the shooter speed.
 */
public class ShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;
    private final DoubleSupplier shootSpeedSupplier;

    /**
     * Constructs a new ShootCommand.
     * @param shooterSubsystem
     * @param shootSpeedSupplier
     */
    public ShootCommand(ShooterSubsystem shooterSubsystem, DoubleSupplier shootSpeedSupplier) {
        this.shooterSubsystem = shooterSubsystem;
        this.shootSpeedSupplier = shootSpeedSupplier;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        this.shooterSubsystem.runRPM(shootSpeedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
