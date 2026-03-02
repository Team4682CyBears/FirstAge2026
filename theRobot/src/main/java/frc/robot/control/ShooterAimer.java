package frc.robot.control;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.common.LookupTableDouble;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterAimer {
  private final DrivetrainSubsystem drivetrain;
  private final SubsystemCollection subsystemCollection;

  private Translation2d targetAdjustment = new Translation2d(0.0, 0.0);

  private double autoYawProfileConstraintsMaxVelocity = 540;
  private double autoYawProfileConstraintsMaxAcceleration = 920;
  private TrapezoidProfile.Constraints autoYawProfileConstraints = new TrapezoidProfile.Constraints(
      autoYawProfileConstraintsMaxVelocity, autoYawProfileConstraintsMaxAcceleration);
  private ProfiledPIDController autoYawPID = new ProfiledPIDController(2.0, 0.0, 0.001, autoYawProfileConstraints);
  private double minYawVelocityRadiansPerSecond = 0.25;
  private double yawVelocityDeadband = 0.01;
  private double yawToleranceDegrees = 3;
  private double shooterRpmTolerance = 100;
  private double kickerRpmTolerance = 100;

  // NOTE: these three LUTs need to have the same min input and max input range. 
  private final double[][] hoodExtensionLookupTableData = { 
      { 1.0000, 0 },
      { 1.3037, 0 },
      { 3.4408, 0.45 },
      { 4.7448, 0.637 },
      { 8.2705, 0.637 } };
  private final double[][] shooterRpmLookupTableData = { 
      { 1.0, 2912 }, 
      { 1.3037, 3000 }, 
      { 4.7448, 4000 },
      { 8.2705, 6500 } };
  private final double[][] kickerRpmLookupTableData = { 
      { 1.0, 2000 },
      { 8.2705, 2000 } };

  private final LookupTableDouble hoodExtensionLookupTable = new LookupTableDouble(hoodExtensionLookupTableData);
  private final LookupTableDouble shooterRpmLookupTable = new LookupTableDouble(shooterRpmLookupTableData);
  private final LookupTableDouble kickerRpmLookupTable = new LookupTableDouble(kickerRpmLookupTableData);

  public ShooterAimer(DrivetrainSubsystem drivetrain, SubsystemCollection subsystemCollection) {
    this.drivetrain = drivetrain;
    this.subsystemCollection = subsystemCollection;
  }

  public Translation2d computePredictedTarget(Translation2d targetFieldTranslation) {
    if (targetFieldTranslation == null) {
      return null;
    }

    // compute robot-relative field velocity
    ChassisSpeeds chassis = drivetrain.getChassisSpeeds(); // robot relative speeds
    Rotation2d robotYaw = drivetrain.getGyroscopeRotation();
    ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(chassis, robotYaw);


    double tof = Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS;

    Translation2d predicted = new Translation2d(
        targetFieldTranslation.getX() - (fieldSpeeds.vxMetersPerSecond * tof) + targetAdjustment.getX(),
        targetFieldTranslation.getY() - (fieldSpeeds.vyMetersPerSecond * tof) + targetAdjustment.getY());

    return predicted;
  }

  /**
   * Apply a small operator adjustment (meters) to aiming target (via d-pad).
   */
  public void applyTargetAdjustment(double dxMeters, double dyMeters) {
    targetAdjustment = targetAdjustment.plus(new Translation2d(dxMeters, dyMeters));
  }

  public void resetTargetAdjustment() {
    targetAdjustment = new Translation2d(0.0, 0.0);
  }

  /**
   * @param targetX meters
   * @param targetY meters
   * @return desired field relative Rotation2d for the robot to face the target
   */
  public Rotation2d getYawToFaceTarget(Translation2d targetTranslation) {
    Pose2d botPos = drivetrain.getRobotPosition();
    double dx = targetTranslation.getX() - botPos.getX();
    double dy = targetTranslation.getY() - botPos.getY();

    double angleRad = Math.atan2(dy, dx);
    return Rotation2d.fromRadians(angleRad);
  }

  /**
   * Returns the desired hood extension (rotations or motor units) for a given
   * distance.
   * The hood subsystem expects a double position (rotations) rather than a servo
   * pulse.
   */
  public double hoodExtensionForDistance(double distanceMeters) {
    // Lookup table should return a double representing hood extension in rotations.
    double ext = hoodExtensionLookupTable.queryTable(distanceMeters);
    // Clamp to mechanical limits (use hood rotation bounds from Constants if
    // available)
    return MathUtil.clamp(ext, Constants.hoodMinPositionRotations, Constants.hoodMaxPositionRotations);
  }

  public double shooterRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(shooterRpmLookupTable.queryTable(distanceMeters), Constants.SHOOTER_MIN_RPM,
        Constants.SHOOTER_MAX_RPM);
  }

  public double kickerRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(kickerRpmLookupTable.queryTable(distanceMeters), Constants.KICKER_MIN_RPM,
        Constants.KICKER_MAX_RPM);
  }

  /**
   * Computes an auto-yaw velocity to command the drivetrain when aiming at a
   * target.
   * If aimTarget is null, defaults to alliance hub.
   */
  public double computeAutoYawVelocityRadiansPerSecond(Translation2d aimTarget) {
    double robotYawRadians = drivetrain.getRobotPosition().getRotation().getRadians();
    if (DriverStation.getAlliance().isEmpty()) {
      System.out.println("WARNING: DriverStation is reporting no alliance!");
    }
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    Translation2d hubPosition = (aimTarget != null) ? aimTarget
        : (alliance == Alliance.Blue ? Constants.blueHubPosition : Constants.redHubPosition);
    double angleToFace = getYawToFaceTarget(hubPosition).getRadians();
    System.out.println("angleToFace: " + Units.radiansToDegrees(angleToFace));

    double error = MathUtil
        .angleModulus(robotYawRadians - angleToFace + Units.degreesToRadians(Constants.shooterYawOffset));
    System.out.println("error: " + Units.radiansToDegrees(error));
    double pidOut = autoYawPID.calculate(error, 0.0);
    double out = (Math.abs(pidOut) > yawVelocityDeadband)
        ? pidOut + Math.signum(pidOut) * minYawVelocityRadiansPerSecond
        : 0.0;
    return out;
  }

  /**
   * Helper to determine if a shot is feasible given ranges/lookup tables.
   */
  public boolean isShotFeasible(Translation2d target) {
    if (target == null)
      return false;
    double distance = drivetrain.getRobotPosition().getTranslation().getDistance(target);
    // check hood lookup bounds (using Constants distances) and shooter rpm bounds
    // assume hood, shooter, and kicker are all defined over the same min and max distance
    return distance >= hoodExtensionLookupTable.getMinInput() && distance <= hoodExtensionLookupTable.getMaxInput();
  }

  /**
   * Check whether current robot yaw, hood extension, shooter velocity and kicker
   * velocity are at
   * the target values
   */
  public boolean isAtPosition(double targetYawRadians, double targetHoodExtension, double targetShooterRpm,
      double targetKickerRpm) {
    double currentYaw = drivetrain.getGyroscopeRotation().getRadians();
    double yawErr = Math.abs(MathUtil.angleModulus(currentYaw - targetYawRadians));
    boolean yawOk = yawErr < Math.toRadians(yawToleranceDegrees); // 3 deg tolerance

    double hoodPos = subsystemCollection.isHoodSubsystemAvailable()
        ? subsystemCollection.getHoodSubsystem().getHoodPosition()
        : Constants.hoodMinPositionRotations;
    boolean hoodOk = Math.abs(hoodPos - targetHoodExtension) < Constants.hoodExtendoTolerance;

    double shooterRpm = subsystemCollection.isShooterSubsystemAvailable()
        ? subsystemCollection.getShooterSubsystem().getRPM()
        : 0.0;
    boolean shooterOk = Math.abs(shooterRpm - targetShooterRpm) < shooterRpmTolerance;

    double kickerRpm = subsystemCollection.isKickerSubsystemAvailable()
        ? subsystemCollection.getKickerSubsystem().getRPM()
        : 0.0;
    boolean kickerOk = Math.abs(kickerRpm - targetKickerRpm) < kickerRpmTolerance;

    return yawOk && hoodOk && shooterOk && kickerOk;
  }
}