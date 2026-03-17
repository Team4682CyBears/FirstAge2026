// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: AutoAimShuttlingCommand.java
// Intent: Command to automatically shuttle balls
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class AutoAimShuttlingCommand extends Command {
  private final SubsystemCollection subsystems;
  private final DrivetrainSubsystem drivetrain;
  private final HoodSubsystem hood;
  private final ShooterSubsystem shooter;
  private final KickerSubsystem kicker;
  private final ShooterAimer aimer;
  private Translation2d hub;


  public static final Translation2d redHubPosition = new Translation2d();

  //
  public AutoAimShuttlingCommand(SubsystemCollection subsystemCollection,
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
    aimer.setDesiredTarget(aimer.getHubPositionFromAlliance());
    subsystems.getDriveTrainPowerSubsystem().setReducedPowerReductionFactor();
  }

  public void execute() {
    // set to shuttle target
    Pose2d robotPosition = drivetrain.getRobotPosition();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Translation2d shuttlePos;
    if (alliance == Alliance.Blue) {
      if (robotPosition.getX() < Constants.FIELD_LENGTH / 2.0) {
        shuttlePos = Constants.blueLeftShuttlePosition;
      } else {
        shuttlePos = Constants.blueRightShuttlePosition;
      }
    } else {
      // red side
      if (robotPosition.getX() < Constants.FIELD_LENGTH / 2.0) {
        shuttlePos = Constants.redLeftShuttlePosition;
      } else {
        shuttlePos = Constants.redRightShuttlePosition;
      }
    }
    aimer.setDesiredTarget(shuttlePos);



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
    drivetrain.setSwerveYawMode(frc.robot.control.SwerveYawMode.JOYSTICK);
    subsystems.getDriveTrainPowerSubsystem().resetPowerReductionFactor();
    shooter.stop();
    hood.retract();
  }

  public boolean isFinished() {
    return false;
  }
}
