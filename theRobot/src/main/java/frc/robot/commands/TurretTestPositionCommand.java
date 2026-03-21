// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: TurretTestPositionCommand.java
// Intent: Command to move turret to a fixed test angle.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.control.TurretAimMode;
import frc.robot.subsystems.TurretSubsystem;

public class TurretTestPositionCommand extends InstantCommand {
  private final TurretSubsystem turret;
  private final double targetRadians;

  public TurretTestPositionCommand(TurretSubsystem turret, double targetRadians) {
    this.turret = turret;
    this.targetRadians = targetRadians;
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    turret.setAimMode(TurretAimMode.MANUAL);
    turret.setTargetAngleRadians(targetRadians);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setAimMode(TurretAimMode.AUTO);
  }
}
