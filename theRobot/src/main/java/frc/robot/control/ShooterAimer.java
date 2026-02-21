package frc.robot.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.common.LookupTableDouble;
import frc.robot.subsystems.DrivetrainSubsystem;

public class ShooterAimer {
  private final DrivetrainSubsystem drivetrain;

  private final double[][] hoodExtensionLookupTableData = { {} };
  private final double[][] shooterRpmLookupTableData = { {} };
  private final double[][] kickerRpmLookupTableData = { {} };

  private final LookupTableDouble hoodExtensionLookupTable = new LookupTableDouble(hoodExtensionLookupTableData);
  private final LookupTableDouble shooterRpmLookupTable = new LookupTableDouble(shooterRpmLookupTableData);
  private final LookupTableDouble kickerRpmLookupTable = new LookupTableDouble(kickerRpmLookupTableData);

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

    Translation2d predicted = new Translation2d(
        targetFieldTranslation.getX() - (fieldVel.getX() * Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS),
        targetFieldTranslation.getY() - (fieldVel.getY() * Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS));

    return predicted;
  }

  public int hoodPulseForDistance(double distanceMeters) {
    return MathUtil.clamp((int) hoodExtensionLookupTable.queryTable(distanceMeters), Constants.HOOD_MIN_PULSE,
        Constants.HOOD_MAX_PULSE);
  }

  public double shooterRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(shooterRpmLookupTable.queryTable(distanceMeters), 0.0, 6784.0);
  }

  public double kickerRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(kickerRpmLookupTable.queryTable(distanceMeters), 0.0, 6000.0);
  }
}