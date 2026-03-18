// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: AutoAimShuttlingCommand.java
// Intent: Command to automatically shuttle balls
// ************************************************************

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.InstalledHardware;
import frc.robot.control.SwerveYawMode;
import frc.robot.control.TurretAimMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoAimShuttlingCommand extends Command {
  private final SubsystemCollection subsystems;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final ShooterAimer aimer;
  private final TurretSubsystem turret;


  //
  public AutoAimShuttlingCommand(SubsystemCollection subsystemCollection,
      ShooterAimer aimer) {
    this.subsystems = subsystemCollection;
    this.hood = subsystemCollection.getHoodSubsystem();
    this.shooter = subsystemCollection.getShooterSubsystem();
    this.aimer = aimer;
    this.turret = subsystemCollection.getTurretSubsystem();
    // We are not declaring drivetrain subsystem as a requirement because it is only
    // setting the swerve yaw mode
    addRequirements(hood, shooter);
  }

  public void initialize() {
    aimer.clearShootingAimTarget();
    subsystems.getDriveTrainPowerSubsystem().setReducedPowerReductionFactor();
    if (subsystems.isDriveTrainSubsystemAvailable()) {
      subsystems.getDriveTrainSubsystem().setSwerveYawMode(
          InstalledHardware.useTurretForAiming ? SwerveYawMode.JOYSTICK : SwerveYawMode.AUTO);
    }
    if (!InstalledHardware.useTurretForAiming && turret != null) {
      turret.setTurretAimMode(TurretAimMode.JOYSTICK);
    }
  }

  public void execute() {
    double ext = aimer.getHoodExtension();
    hood.setExtendoPosition(ext);
    double shooterRpm = aimer.getShooterRPM();
    shooter.runRPM(shooterRpm);
    SmartDashboard.putNumber("Calc Shooter Speed", shooterRpm);
    SmartDashboard.putNumber("Calced Hood Extendo", ext);
  }

  public void end(boolean interrupted) {
    aimer.clearShootingAimTarget();
    subsystems.getDriveTrainPowerSubsystem().resetPowerReductionFactor();
    shooter.stop();
    hood.retract();
    if (subsystems.isDriveTrainSubsystemAvailable()) {
      subsystems.getDriveTrainSubsystem().setSwerveYawMode(SwerveYawMode.JOYSTICK);
    }
    if (turret != null && InstalledHardware.useTurretForAiming) {
      turret.setTurretAimMode(TurretAimMode.AUTO);
    }
  }

  public boolean isFinished() {
    return false;
  }
}
