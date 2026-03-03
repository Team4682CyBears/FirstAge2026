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

  private final boolean doCompensateForRotation = false;

  private Translation2d desiredTarget = null;
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
  private final double[][] tofLookupTableData = {
      { 1.0000, 0.923 },
      { 1.3037, 0.923 },
      { 3.4408, 1.193 },
      { 4.7448, 1.300 },
      { 8.2705, 2.315 } };

  private final LookupTableDouble hoodExtensionLookupTable = new LookupTableDouble(hoodExtensionLookupTableData);
  private final LookupTableDouble shooterRpmLookupTable = new LookupTableDouble(shooterRpmLookupTableData);
  private final LookupTableDouble kickerRpmLookupTable = new LookupTableDouble(kickerRpmLookupTableData);
  private final LookupTableDouble tofLookupTable = new LookupTableDouble(tofLookupTableData);

  public ShooterAimer(DrivetrainSubsystem drivetrain, SubsystemCollection subsystemCollection) {
    this.drivetrain = drivetrain;
    this.subsystemCollection = subsystemCollection;
  }

  /**
   * Apply a small operator adjustment (meters) to aiming target (via d-pad).
   */
  public void applyTargetAdjustment(double dxMeters, double dyMeters) {
    targetAdjustment = targetAdjustment.plus(new Translation2d(dxMeters, dyMeters));
  }

  /**
   * Clear the shooting target
   */
  public void clearShootingAimTarget() {
    this.desiredTarget = null;
  }

  /**
   * Compute the predicted target accounting for targetAdjustments, velocities,
   * and time of flight
   * 
   * @return the predicted target
   */
  public Translation2d computePredictedTarget() {
    if (desiredTarget == null) {
      return null;
    }

    // compute robot-relative field velocity
    ChassisSpeeds fieldSpeeds = drivetrain.getChassisSpeedsFieldCentric();
    Translation2d fieldSpeedsTranslation = new Translation2d(fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond);
    if (doCompensateForRotation) {
      // TODO test this on-robot. Not sure about direction of the velocity. Might be -90?
      // velocity due to rotation = angular velocity * shooterOffset rotated by 90
      // degrees (it's orthogonal to the radius).
      System.out.println("field speeds " + fieldSpeeds);
      Translation2d rotationalVelocityRobotCentric = Constants.shooterOffsetFromCenterOfRobot
          .times(fieldSpeeds.omegaRadiansPerSecond).rotateBy(Rotation2d.fromDegrees(90));
      Translation2d rotationalVelocityFieldCentric = rotationalVelocityRobotCentric
          .rotateBy(drivetrain.getGyroscopeRotation());
      fieldSpeedsTranslation = fieldSpeedsTranslation.plus(rotationalVelocityFieldCentric);
    }

    double tof = Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS;

    Translation2d predicted = desiredTarget.minus(fieldSpeedsTranslation.times(tof)).plus(targetAdjustment);
    return predicted;
  }

  /**
   * Computes an auto-yaw velocity to command the drivetrain when aiming at a
   * target.
   * If aimTarget is null, defaults to alliance hub.
   */
  public double computeAutoYawVelocityRadiansPerSecond() {
    double robotYawRadians = drivetrain.getRobotPosition().getRotation().getRadians();
    double angleToFace = getYawToFaceTarget().getRadians();
    System.out.println("angleToFace: " + Units.radiansToDegrees(angleToFace));

    double error = MathUtil
        .angleModulus(robotYawRadians - angleToFace);
    System.out.println("error: " + Units.radiansToDegrees(error));
    double pidOut = autoYawPID.calculate(error, 0.0);
    double out = (Math.abs(pidOut) > yawVelocityDeadband)
        ? pidOut + Math.signum(pidOut) * minYawVelocityRadiansPerSecond
        : 0.0;
    return out;
  }

  /**
   * get the desired target
   * 
   * @return desired target
   */
  public Translation2d getDesiredTarget() {
    return desiredTarget;
  }

  /**
   * get the distance to the predicted target
   * 
   * @return distance
   */
  public double getDistanceToPredictedTarget() {
    Translation2d predicted = computePredictedTarget();
    return drivetrain.getRobotPosition().getTranslation().getDistance(predicted);
  }

  /**
   * get the hub position from the alliance
   * defaults to blue alliance if DriveStation.getAlliance() is not available
   * 
   * @return
   */
  public Translation2d getHubPositionFromAlliance() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    return alliance == Alliance.Blue ? Constants.blueHubPosition : Constants.redHubPosition;
  }

  /**
   * @param targetX meters
   * @param targetY meters
   * @return desired field relative Rotation2d for the robot to face the target
   *         accounts for shooterYawOffset relative to robot
   */
  public Rotation2d getYawToFaceTarget() {
    Translation2d targetTranslation = computePredictedTarget();
    Pose2d botPos = drivetrain.getRobotPosition();
    double dx = targetTranslation.getX() - botPos.getX();
    double dy = targetTranslation.getY() - botPos.getY();

    double angleRad = Math.atan2(dy, dx);
    return Rotation2d.fromRadians(angleRad).plus(Constants.shooterYawOffset);
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

  /**
   * Helper to determine if a shot is feasible given ranges/lookup tables.
   */
  public boolean isShotFeasible(Translation2d target) {
    if (target == null)
      return false;
    double distance = drivetrain.getRobotPosition().getTranslation().getDistance(target);
    // check hood lookup bounds (using Constants distances) and shooter rpm bounds
    // assume hood, shooter, and kicker are all defined over the same min and max
    // distance
    return distance >= hoodExtensionLookupTable.getMinInput() && distance <= hoodExtensionLookupTable.getMaxInput();
  }

  /**
   * helper to get the kicker RPM for a given distance
   * 
   * @param distanceMeters
   * @return kicker RPM
   */
  public double kickerRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(kickerRpmLookupTable.queryTable(distanceMeters), Constants.KICKER_MIN_RPM,
        Constants.KICKER_MAX_RPM);
  }

  /**
   * ressets the targetAdjustment back to 0,0
   */
  public void resetTargetAdjustment() {
    targetAdjustment = new Translation2d(0.0, 0.0);
  }

  /**
   * sets the desired target
   * 
   * @param target
   */
  public void setDesiredTarget(Translation2d target) {
    this.desiredTarget = target;
  }

  /**
   * helper to get the shooter RPM for a given distance
   * 
   * @param distanceMeters
   * @return shooter RPM
   */
  public double shooterRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(shooterRpmLookupTable.queryTable(distanceMeters), Constants.SHOOTER_MIN_RPM,
        Constants.SHOOTER_MAX_RPM);
  }

  public double tofForDisatnce(double distanceMeters){
    return tofLookupTable.queryTable(distanceMeters);
  }

  /**
   * Helper to update the chassis speeds with the most recent autoYaw velocity
   * 
   * @param chassisSpeeds
   * @return updated chassisSpeeds
   */
  public ChassisSpeeds updateChassisSpeedsWithAutoYaw(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        computeAutoYawVelocityRadiansPerSecond());
    return newChassisSpeeds;
  }
}