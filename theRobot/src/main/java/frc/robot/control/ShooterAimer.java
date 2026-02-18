package frc.robot.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.common.LookupTableDouble;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ShooterAimer {
  private final DrivetrainSubsystem drivetrain;

  private final double[][] hoodAngleLookupTable = { {} };
  private final double[][] shooterRpmLookupTable = { {} };

  private final LookupTableDouble hoodAngleLookupTableImpl = new LookupTableDouble(hoodAngleLookupTable);
  private final LookupTableDouble shooterRpmLookupTableImpl = new LookupTableDouble(shooterRpmLookupTable);

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

    return MathUtil.clamp((int) hoodAngleLookupTableImpl.queryTable(distanceMeters), Constants.HOOD_MIN_PULSE,
        Constants.HOOD_MAX_PULSE);
  }

  public double shooterRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(shooterRpmLookupTableImpl.queryTable(distanceMeters), 0.0, 6784);
  }
}