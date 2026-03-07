// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: AutoAimMovingCommand.java
// Intent: Command to automatically aim at a moving target
// ************************************************************

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.control.Constants;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoAimMovingCommand extends Command {
  private final SubsystemCollection subsystems;
  private final DrivetrainSubsystem drivetrain;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final ShooterAimer aimer;
  private Translation2d hub;

  //
  public AutoAimMovingCommand(SubsystemCollection subsystemCollection,
      ShooterAimer aimer) {
    this.subsystems = subsystemCollection;
    this.drivetrain = subsystemCollection.getDriveTrainSubsystem();
    this.hood = subsystemCollection.getHoodSubsystem();
    this.shooter = subsystemCollection.getShooterSubsystem();
    this.kicker = subsystemCollection.getKickerSubsystem();
    this.aimer = aimer;
    // We are not declaring drivetrain subsystem as a requirement because it is only
    // setting the swerve yaw mode
    //TODO kicker needs to be added as a requirement once testing is done.
    addRequirements(hood, shooter);
  }

  public void initialize() {
    drivetrain.setSwerveYawMode(frc.robot.control.SwerveYawMode.AUTO);

    Alliance alliance = DriverStation.getAlliance().get();
    hub = (alliance == Alliance.Blue) ? Constants.blueHubPosition : Constants.redHubPosition;

    Translation2d predicted = aimer.computePredictedTarget(hub);
    if (predicted != null) {
      drivetrain.setShootingAimTarget(predicted);
    }

    subsystems.getDriveTrainPowerSubsystem().setReducedPowerReductionFactor();
  }

  public void execute() {
    Translation2d predicted = aimer.computePredictedTarget(hub);
    if (predicted != null) {
      double distance = drivetrain.getRobotPosition().getTranslation().getDistance(predicted);
      double ext = aimer.hoodExtensionForDistance(distance);
      hood.setExtendoPosition(ext);
      double shooterRpm = aimer.shooterRpmForDistance(distance);
      shooter.runRPM(shooterRpm);
      SmartDashboard.putNumber("Calc Shooter Speed", shooterRpm);
      SmartDashboard.putNumber("Calced Hood Extendo", ext);
      // double kickerRpm = aimer.kickerRpmForDistance(distance);
      // kicker.runRPM(kickerRpm);
    }
  }

  public void end(boolean interrupted) {
    drivetrain.clearShootingAimTarget();
    drivetrain.setSwerveYawMode(frc.robot.control.SwerveYawMode.JOYSTICK);
    subsystems.getDriveTrainPowerSubsystem().resetPowerReductionFactor();
    shooter.stop();
  }

  public boolean isFinished() {
    return false;
  }
}
