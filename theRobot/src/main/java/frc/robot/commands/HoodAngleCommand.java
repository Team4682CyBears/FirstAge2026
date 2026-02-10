// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: HoodAngleCommand.java
// Intent: command to set hood angle and extendo position
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class HoodAngleCommand extends Command {
    private final HoodSubsystem hoodSubsystem;
    private final IntSupplier angleSupplier;
    private final IntSupplier extendoSupplier;

    public HoodAngleCommand(HoodSubsystem hoodSubsystem, IntSupplier angleSupplier, IntSupplier extendoSupplier) {
        this.hoodSubsystem = hoodSubsystem;
        this.angleSupplier = angleSupplier;
        this.extendoSupplier = extendoSupplier;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        this.hoodSubsystem.setAnglePosition(angleSupplier.getAsInt());
        this.hoodSubsystem.setExtendoPosition(extendoSupplier.getAsInt());
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
