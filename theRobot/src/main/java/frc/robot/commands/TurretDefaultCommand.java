// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: TurretDefaultCommand.java
// Intent: Default command to auto-aim the turret using ShooterAimer
// ************************************************************

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.TurretAimMode;
import frc.robot.subsystems.TurretSubsystem;

public class TurretDefaultCommand extends Command {
  private final SubsystemCollection subsystems;
  private final TurretSubsystem turret;

  public TurretDefaultCommand(SubsystemCollection subsystems) {
    this.subsystems = subsystems;
    this.turret = subsystems.getTurretSubsystem();
    addRequirements(turret);
  }

  @Override
  public void initialize() {
    if (turret != null) {
      turret.setAimMode(TurretAimMode.AUTO);
    }
  }

  @Override
  public void execute() {
    if (!subsystems.isShooterAimerAvailable()) {
      if (turret != null) {
        turret.setAimMode(TurretAimMode.MANUAL);
        turret.setTargetAngleRadians(turret.getAngleRotation2d().getRadians());
      }
      return;
    }

    if (turret != null && turret.getAimMode() == TurretAimMode.AUTO) {
      ShooterAimer aimer = subsystems.getShooterAimer();
      aimer.calculate();
      turret.setTargetAngleRadians(aimer.getDesiredTurretAngleRadians());
    }
  }
}
