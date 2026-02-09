package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.control.Constants;

public class ShooterAimer {
  private final DrivetrainSubsystem drivetrain;

  public ShooterAimer(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
  }

  public Translation2d computePredictedTarget(Translation2d targetFieldTranslation) {
    if (targetFieldTranslation == null) {
      return null;
    }

    ChassisSpeeds chassis = drivetrain.getChassisSpeeds(); // robot relative speeds

    Rotation2d robotYaw = drivetrain.getGyroscopeRotation();
    Translation2d fieldVel = new Translation2d(chassis.vxMetersPerSecond, chassis.vyMetersPerSecond)
        .rotateBy(robotYaw);

    double distance = drivetrain.getRobotPosition().getTranslation().getDistance(targetFieldTranslation);

    double projectileSpeed = Constants.PROJECTILE_SPEED_METERS_PER_SECOND;
    double flightTime = projectileSpeed > 0.001 ? (distance / projectileSpeed) : 0.0;

    Translation2d predicted = new Translation2d(
        targetFieldTranslation.getX() - (fieldVel.getX() * flightTime),
        targetFieldTranslation.getY() - (fieldVel.getY() * flightTime));

    return predicted;
  }

  public int hoodPulseForDistance(double distanceMeters) {
    double t = (distanceMeters - Constants.HOOD_MIN_DISTANCE_METERS)
        / (Constants.HOOD_MAX_DISTANCE_METERS - Constants.HOOD_MIN_DISTANCE_METERS);
    t = Math.max(0.0, Math.min(1.0, t));
    int pulse = (int) Math.round(Constants.HOOD_MIN_PULSE + t * (Constants.HOOD_MAX_PULSE - Constants.HOOD_MIN_PULSE));
    return pulse;
  }
}