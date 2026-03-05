// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: HoodAngleCommand.java
// Intent: command to set hood angle and extendo position
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

/**
 * Command to set the hood extendo position.
 */
public class HoodAngleCommand extends Command {
    private final HoodSubsystem hoodSubsystem;
    private final DoubleSupplier extendoSupplier;

    /**
     * Constructs a new HoodAngleCommand.
     *
     * @param hoodSubsystem   the hood subsystem to control
     * @param extendoSupplier a supplier that provides the target extendo position
     */
    public HoodAngleCommand(HoodSubsystem hoodSubsystem, DoubleSupplier extendoSupplier) {
        this.hoodSubsystem = hoodSubsystem;
        this.extendoSupplier = extendoSupplier;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        this.hoodSubsystem.setExtendoPosition(extendoSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
