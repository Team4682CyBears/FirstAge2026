package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.control.Constants;
import frc.robot.control.ShooterAimer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoAimMovingCommand extends Command {
  private final DrivetrainSubsystem drivetrain;
  private final HoodSubsystem hood;
  private final ShooterAimer aimer;

  public AutoAimMovingCommand(DrivetrainSubsystem drivetrain, HoodSubsystem hood, ShooterAimer aimer) {
    this.drivetrain = drivetrain;
    this.hood = hood;
    this.aimer = aimer;
    addRequirements(hood);
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
      int pulse = aimer.hoodPulseForDistance(distance);
      hood.setAnglePosition(pulse);
    }
  }

  public void end(boolean interrupted) {
    drivetrain.clearShootingAimTarget();
    drivetrain.setSwerveYawMode(frc.robot.control.SwerveYawMode.JOYSTICK);
  }

  public boolean isFinished() {
    return false;
  }
}
