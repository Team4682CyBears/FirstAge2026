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
  private final DrivetrainSubsystem drivetrain;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final ShooterAimer aimer;

  //
  public AutoAimMovingCommand(SubsystemCollection subsystemCollection,
      ShooterAimer aimer) {
    this.drivetrain = subsystemCollection.getDriveTrainSubsystem();
    this.hood = subsystemCollection.getHoodSubsystem();
    this.shooter = subsystemCollection.getShooterSubsystem();
    this.kicker = subsystemCollection.getKickerSubsystem();
    this.aimer = aimer;
    // We are not declaring drivetrain subsystem as a requirement because it is only
    // setting the swerve yaw mode
    addRequirements(hood, shooter);
  }

  public void initialize() {
    drivetrain.setSwerveYawMode(frc.robot.control.SwerveYawMode.AUTO);
  }

  public void execute() {
    Alliance alliance = DriverStation.getAlliance().get();
    Translation2d hub = (alliance == Alliance.Blue) ? Constants.blueHubPosition : Constants.redHubPosition;

    Translation2d predicted = aimer.computePredictedTarget(hub);
    if (predicted != null) {
      drivetrain.setShootingAimTarget(predicted);

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
    shooter.stop();
    hood.retract();
  }

  public boolean isFinished() {
    return false;
  }
}
