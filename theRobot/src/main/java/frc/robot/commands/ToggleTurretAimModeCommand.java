// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ToggleTurretAimModeCommand.java
// Intent: Toggle turret aim mode between AUTO and MANUAL.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.TurretAimMode;
import frc.robot.subsystems.TurretSubsystem;

/**
 * Toggle the turret aim mode. When switching to MANUAL, hold the current angle.
 */
public class ToggleTurretAimModeCommand extends Command {
    private final TurretSubsystem turret;

    public ToggleTurretAimModeCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        if (turret == null) {
            return;
        }
        TurretAimMode currentMode = turret.getAimMode();
        if (currentMode == TurretAimMode.AUTO) {
            turret.setAimMode(TurretAimMode.MANUAL);
            turret.setTargetAngleRadians(turret.getAngleRadians());
        } else {
            turret.setAimMode(TurretAimMode.AUTO);
        }
    }
}
