// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: AutoAimMovingCommand.java
// Intent: Command to automatically aim at a moving target
// ************************************************************

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.TurretAimMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAimMovingCommand extends Command {
  private final SubsystemCollection subsystems;
  private final TurretSubsystem turret;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final ShooterAimer aimer;

  //
  public AutoAimMovingCommand(SubsystemCollection subsystemCollection,
      ShooterAimer aimer) {
    this.subsystems = subsystemCollection;
  this.turret = subsystemCollection.getTurretSubsystem();
    this.hood = subsystemCollection.getHoodSubsystem();
    this.shooter = subsystemCollection.getShooterSubsystem();
    this.aimer = aimer;
    // We are not declaring drivetrain subsystem as a requirement because it is only
    // setting the swerve yaw mode
    //TODO kicker needs to be added as a requirement once testing is done.
    addRequirements(hood, shooter);
    if (turret != null) {
      addRequirements(turret);
    }
  }

  public void initialize() {
    turret.setTurretAimMode(TurretAimMode.AUTO);
    aimer.setDesiredTarget(aimer.getHubPositionFromAlliance());
    subsystems.getDriveTrainPowerSubsystem().setReducedPowerReductionFactor();
  }

  public void execute() {
    double ext = aimer.getHoodExtension();
    hood.setExtendoPosition(ext);
    double shooterRpm = aimer.getShooterRPM();
    shooter.runRPM(shooterRpm);
    SmartDashboard.putNumber("Calc Shooter Speed", shooterRpm);
    SmartDashboard.putNumber("Calced Hood Extendo", ext);
    // double kickerRpm = aimer.kickerRpmForDistance(distance);
    // kicker.runRPM(kickerRpm);
  }

  public void end(boolean interrupted) {
    aimer.clearShootingAimTarget();
    turret.setTurretAimMode(TurretAimMode.AUTO);
    subsystems.getDriveTrainPowerSubsystem().resetPowerReductionFactor();
    shooter.stop();
    hood.retract();
  }

  public boolean isFinished() {
    return false;
  }
}
