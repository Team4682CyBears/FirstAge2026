// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShooterAimer.java
// Intent: Helper class to compute aiming parameters for shooting at a target
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.common.LookupTableDouble;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ShooterAimer {
  private final DrivetrainSubsystem drivetrain;
  private final SubsystemCollection subsystemCollection;

  // NOTE protected flags and methods below exposed for unit testing purposes
  // otherwise, they would have been private
  protected boolean doCompensateForRotation = true;
  protected boolean displayDiagnostics = false;

  private Translation2d desiredTarget = null;
  private Translation2d targetAdjustment = new Translation2d(0.0, 0.0);
  private Translation2d predictedTarget = null;
  private double distance = 0.0;
  private double shooterRPM = 0.0;
  private double kickerRPM = 0.0;
  private double hoodExtension = 0.0;
  private Rotation2d autoYaw = new Rotation2d();
  private double autoYawVelocity = 0.0;
  private double predictedTimeOfFlight = Constants.DEFAULT_PROJECTILE_TIME_OF_FLIGHT_SECONDS;

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
      { 1.0000, 0.8 },
      { 1.3037, 0.12 },
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

  /*
   * A class to handle all the shooter aiming related calculations
   * shooterRPM, kickerRPM, autoYaw angle, autoYaw velocity, hoodExtension
   * requires drivetrain to call calculate() method in periodic when in auto-yaw mode. 
   */
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
   * Calculates all the settings related to shooting
   * predicted target, distance, hood angle, shooter speed, auto yaw angle, yaw velocities,
   * and kicker speed
   * Intended to be called every tick when auto aiming is in use
   */
  public void calculate(){
    if (desiredTarget != null){
      predictedTarget = computePredictedTarget();
      autoYaw = computeYawToFaceTarget();
      autoYawVelocity = computeAutoYawVelocityRadiansPerSecond();
      distance = computeDistanceToPredictedTarget();
      shooterRPM = shooterRpmForDistance(distance);
      kickerRPM = kickerRpmForDistance(distance);
      hoodExtension = hoodExtensionForDistance(distance);
      predictedTimeOfFlight = tofForDistance(distance);
      if (displayDiagnostics) {
      System.out.println("Shooter Aimer updated!");
      System.out.println("Predicted Target " + predictedTarget);
      System.out.println("AutoYaw " + autoYaw);
      System.out.println("AutoYawVelocity " + autoYawVelocity);
      System.out.println("Distance " + distance);
      System.out.println("PredictedTimeOfFlight " + predictedTimeOfFlight);
      }
    }
  }

  /**
   * Clear the shooting target
   */
  public void clearShootingAimTarget() {
    desiredTarget = null;
    predictedTarget = null;
    shooterRPM = 0.0;
    kickerRPM = 0.0;
    hoodExtension = 0.0;
    autoYaw = new Rotation2d();
    autoYawVelocity = 0.0;
    predictedTimeOfFlight = Constants.DEFAULT_PROJECTILE_TIME_OF_FLIGHT_SECONDS;
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
   * Get pre-calculated autoYaw
   * @return autoYaw
   */
  public Rotation2d getAutoYaw(){
    return autoYaw;
  }

  /**
   * Get pre-calculated autoYawVelocity
   * @return autoYawVelocity in radians per second
   */
  public double getAutoYawVelocityRadiansPerSecond() {
    return autoYawVelocity;
  }

  /**
   * Get pre-calculated distance from predicted target
   * @return distance (m)
   */
  public double getDistanceForPredictedTarget() {
    return distance;
  }

  /**
   * Get pre-calculated hood extension
   * @return hood extension
   */
  public double getHoodExtension() {
    return hoodExtension;
  }

  /**
   * Get pre-calculated kicker RPM
   * @return kicker RPM
   */
  public double getKickerRPM() {
    return kickerRPM;
  }

  /**
   * Get kicker min speed 
   * @return kicker min speed in RPM
   */
  public double getMinKickerSpeedRPM() {
    return kickerRpmLookupTable.getMinOutput();
  }

  /**
   * Get shooter min speed
   * @return shooter min speed in RPM
   */
  public double getMinShooterSpeedRPM() {
    return shooterRpmLookupTable.getMinOutput();
  }

  /**
   * Get pre-computed predicted target
   * @return predicted target
   */
  public Translation2d getPredictedTarget(){
    return predictedTarget;
  }

  /**
   * Get pre-computed shooterRPM
   * @return shooterRPM
   */
  public double getShooterRPM() {
    return shooterRPM;
  }

  /**
   * Get pre-computed time of flight
   * @return time of flight (seconds)
   */
  public double getTimeOfFlight() {
    return predictedTimeOfFlight;
  }

  /**
   * Check whether current robot yaw, hood extension, shooter velocity and kicker
   * velocity are at
   * the target values
   */
  public boolean isAtPosition() {
    Rotation2d currentYaw = drivetrain.getGyroscopeRotation();
    double yawErr = Math.abs(MathUtil.angleModulus(currentYaw.minus(autoYaw).getRadians()));
    boolean yawOk = yawErr < Math.toRadians(yawToleranceDegrees); // 3 deg tolerance

    double hoodPos = subsystemCollection.isHoodSubsystemAvailable()
        ? subsystemCollection.getHoodSubsystem().getHoodPosition()
        : Constants.hoodMinPositionRotations;
    boolean hoodOk = Math.abs(hoodPos - hoodExtension) < Constants.hoodExtendoTolerance;

    double currentShooterRPM = subsystemCollection.isShooterSubsystemAvailable()
        ? subsystemCollection.getShooterSubsystem().getRPM()
        : 0.0;
    boolean shooterOk = Math.abs(currentShooterRPM - shooterRPM) < shooterRpmTolerance;

    double currentKickerRpm = subsystemCollection.isKickerSubsystemAvailable()
        ? subsystemCollection.getKickerSubsystem().getRPM()
        : 0.0;
    boolean kickerOk = Math.abs(currentKickerRpm - kickerRPM) < kickerRpmTolerance;

    return yawOk && hoodOk && shooterOk && kickerOk;
  }

  /**
   * Helper to determine if a shot is feasible given ranges/lookup tables.
   */
  public boolean isShotFeasible() {
    if (predictedTarget == null)
      return false;
    // check hood lookup bounds (using Constants distances) and shooter rpm bounds
    // assume hood, shooter, and kicker are all defined over the same min and max
    // distance
    return distance >= hoodExtensionLookupTable.getMinInput() && distance <= hoodExtensionLookupTable.getMaxInput();
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
   * Helper to update the chassis speeds with the most recent autoYaw velocity
   * 
   * @param chassisSpeeds
   * @return updated chassisSpeeds
   */
  public ChassisSpeeds updateChassisSpeedsWithAutoYaw(ChassisSpeeds chassisSpeeds) {
    ChassisSpeeds newChassisSpeeds = new ChassisSpeeds(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        autoYawVelocity);
    return newChassisSpeeds;
  }

  //// PRIVATE METHODS ///

  /**
   * Computes an auto-yaw velocity to command the drivetrain when aiming at a
   * target.
   * If aimTarget is null, defaults to alliance hub.
   */
  private double computeAutoYawVelocityRadiansPerSecond() { //USED
    double robotYawRadians = drivetrain.getRobotPosition().getRotation().getRadians();
    double angleToFace = autoYaw.getRadians();

    double error = MathUtil
        .angleModulus(robotYawRadians - angleToFace);
    double pidOut = autoYawPID.calculate(error, 0.0);
    double out = (Math.abs(pidOut) > yawVelocityDeadband)
        ? pidOut + Math.signum(pidOut) * minYawVelocityRadiansPerSecond
        : 0.0;
    return out;
  }

  /**
   * get the distance to the predicted target
   * 
   * @return distance
   */
  private double computeDistanceToPredictedTarget() { // USED
    return drivetrain.getRobotPosition().getTranslation().getDistance(predictedTarget);
  }

  /**
   * Compute the predicted target accounting for targetAdjustments, velocities,
   * and time of flight
   * 
   * @return the predicted target
   */
  protected Translation2d computePredictedTarget() { //USED
    if (desiredTarget == null) {
      return null;
    }

    // compute robot-relative field velocity
    ChassisSpeeds fieldSpeeds = drivetrain.getChassisSpeedsFieldCentric();
    Translation2d fieldSpeedsTranslation = new Translation2d(fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond);
    if (doCompensateForRotation) {
      // TODO test this on-robot. Not sure about direction of the velocity.
      // construct expected rot velocity vector
      Translation2d rotVelocityVector = new Translation2d(
          // magnitude is the shooter distance from center of robot * the rotational
          // velocity
          Constants.shooterOffsetFromCenterOfRobot.getNorm() * Math.abs(fieldSpeeds.omegaRadiansPerSecond),
          // angle w.r.t. field is orthogonal (90 degrees) to the vector from shooter to
          // robot center
          // direction depends on the rotation direction
          drivetrain.getGyroscopeRotation()
              .minus(Constants.shooterYawOffset) // take out shooter yaw offset
              .minus(Rotation2d.fromDegrees(Math.copySign(-90, fieldSpeeds.omegaRadiansPerSecond))));
      System.out.println("ShooterAimer|ComputePredictedTarget->RotVelocityVector " + rotVelocityVector);
      fieldSpeedsTranslation = fieldSpeedsTranslation.plus(rotVelocityVector);
    }

    // NOTE: This function is called before the timeOfFlight is updated, 
    // It uses the timeOfFlight from the last tick because otherwise, 
    // this would be a circular dependency. 
    Translation2d predicted = desiredTarget.minus(fieldSpeedsTranslation.times(predictedTimeOfFlight)).plus(targetAdjustment);
    return predicted;
  }

  /**
   * @param targetX meters
   * @param targetY meters
   * @return desired field relative Rotation2d for the robot to face the target
   *         accounts for shooterYawOffset relative to robot
   */
  protected Rotation2d computeYawToFaceTarget() { //USED
    Pose2d botPos = drivetrain.getRobotPosition();
    double dx = predictedTarget.getX() - botPos.getX();
    double dy = predictedTarget.getY() - botPos.getY();

    double angleRad = Math.atan2(dy, dx);
    return Rotation2d.fromRadians(angleRad).plus(Constants.shooterYawOffset);
  }

  /**
   * Returns the desired hood extension (rotations or motor units) for a given
   * distance.
   * The hood subsystem expects a double position (rotations) rather than a servo
   * pulse.
   */
  protected double hoodExtensionForDistance(double distanceMeters) { //USED
    // Lookup table should return a double representing hood extension in rotations.
    double ext = hoodExtensionLookupTable.queryTable(distanceMeters);
    // Clamp to mechanical limits (use hood rotation bounds from Constants if
    // available)
    return MathUtil.clamp(ext, Constants.hoodMinPositionRotations, Constants.hoodMaxPositionRotations);
  }

  /**
   * helper to get the kicker RPM for a given distance
   * 
   * @param distanceMeters
   * @return kicker RPM
   */
  private double kickerRpmForDistance(double distanceMeters) { //USED
    return MathUtil.clamp(kickerRpmLookupTable.queryTable(distanceMeters), Constants.KICKER_MIN_RPM,
        Constants.KICKER_MAX_RPM);
  }

  /**
   * helper to get the shooter RPM for a given distance
   * 
   * @param distanceMeters
   * @return shooter RPM
   */
  protected double shooterRpmForDistance(double distanceMeters) { //USED
    return MathUtil.clamp(shooterRpmLookupTable.queryTable(distanceMeters), Constants.SHOOTER_MIN_RPM,
        Constants.SHOOTER_MAX_RPM);
  }

  private double tofForDistance(double distanceMeters) { //USED
    return tofLookupTable.queryTable(distanceMeters);
  }
}