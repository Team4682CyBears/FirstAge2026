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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterAimer {
  private final DrivetrainSubsystem drivetrain;
  private final SubsystemCollection subsystemCollection;

  // NOTE protected flags and methods below exposed for unit testing purposes
  // otherwise, they would have been private
  protected boolean doCompensateForRotation = true;
  protected boolean displayDiagnostics = false;

  private Translation2d desiredTarget = null;
  private Translation2d predictedTarget = null;
  private double distance = 0.0;
  private double shooterRPM = 0.0;
  private boolean shouldRevShooter = false;
  private double kickerRPM = 0.0;
  private double hoodExtension = 0.0;
  private Rotation2d autoYaw = new Rotation2d();
  private Rotation2d desiredTurretAngle = new Rotation2d();
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
      { 1.0000, 0.00 },
      { 1.3037, 0.00 },
      { 3.4408, 0.25 },
      { 4.7448, 0.38 },
      { 8.2705, 0.50 } };
  private final double[][] shooterRpmLookupTableData = {
      { 1.0, 2912 },
      { 1.3037, 3000 },
      { 4.7448, 4200 },
      { 8.2705, 5000 } };
  private final double[][] kickerRpmLookupTableData = {
      { 1.0, 2000 },
      { 8.2705, 2000 } };
  private final double[][] tofLookupTableData = {
      { 1.0000, 0.913 },
      { 1.3037, 0.960 },
      { 3.4408, 1.222 },
      { 4.7448, 1.332 },
      { 8.2705, 1.729 } };

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
   * Calculates all the settings related to shooting
   * predicted target, distance, hood angle, shooter speed, auto yaw angle, yaw velocities,
   * and kicker speed
   * Intended to be called every tick when auto aiming is in use
   */
  public void calculate(){
    desiredTarget = getAimTarget();
    if (desiredTarget == null) {
      desiredTarget = getHubPositionFromAlliance();
    }
    if (desiredTarget != null){
      predictedTarget = computePredictedTarget();
      autoYaw = computeYawToFaceTarget();
      desiredTurretAngle = computeTurretAngle();
      autoYawVelocity = computeAutoYawVelocityRadiansPerSecond();
      distance = computeDistanceToPredictedTarget();
      shooterRPM = shooterRpmForDistance(distance);
      kickerRPM = kickerRpmForDistance(distance);
      hoodExtension = hoodExtensionForDistance(distance);
      predictedTimeOfFlight = tofForDistance(distance);
      SmartDashboard.putNumber("SA distance", distance);
      SmartDashboard.putString("SA desired target", desiredTarget.toString());
      SmartDashboard.putString("SA Predicted Target", predictedTarget.toString());
      SmartDashboard.putNumber("SA Turret Angle", desiredTurretAngle.getDegrees());
      if (displayDiagnostics) {
      System.out.println("Shooter Aimer updated!");
      System.out.println("Predicted Target " + predictedTarget);
      System.out.println("AutoYaw " + autoYaw);
      System.out.println("AutoYawVelocity " + autoYawVelocity);
      System.out.println("Turret Angle" + desiredTurretAngle);
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
    shouldRevShooter = false;
    kickerRPM = 0.0;
    hoodExtension = 0.0;
    autoYaw = new Rotation2d();
    desiredTurretAngle = new Rotation2d();
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
   * Desired turret angle (radians) relative to the robot's forward direction.
   */
  public double getDesiredTurretAngleRadians() {
    return desiredTurretAngle.getRadians();
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
   * Get pre-computed shouldRevShooter
   * @return shouldRevShooter
   */
  public boolean getShouldRevShooter(){
    return shouldRevShooter;
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
    Rotation2d currentYaw = getCurrentYawRotation();
    // TODO not sure this is correct, since autoYaw != the total yaw the turret yaw is targeting
    // I think if useTurretForAiming==true, we should return 
    // whether the turret ange == desiredTurretAngle
    // if useTurretForAiming==false, we should return 
    // whether getGyroscopeRotation == autoYaw
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
   * Aim target chosen based on alliance and hub X position.
   * Score when we are at/inside the hub X (blue: <= hub X, red: >= hub X);
   * otherwise aim at the shuttle target.
   * side effect: if canScoreFromHere is true, sets shouldRevShooter to true.
   * TODO fix this to be handled cleanly, rather than via a side-effect.
   */
  private Translation2d getAimTarget() {
    Pose2d robotPose = drivetrain.getRobotPosition();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double hubX = alliance == Alliance.Blue
        ? Constants.blueHubPosition.getX()
        : Constants.redHubPosition.getX();
    boolean canScoreFromHere = alliance == Alliance.Blue
        ? robotPose.getX() <= hubX
        : robotPose.getX() >= hubX;

    if (canScoreFromHere) {
      shouldRevShooter = true;
      return getHubPositionFromAlliance();
    }

  // Field coordinates are in the WPILib global frame (origin at blue driver
  // station, +X toward red, +Y toward the left side of the blue driver station).
  // This means "left" here is always relative to the blue driver station view.
  // We still select red/blue targets based on alliance below.
  boolean isLeftSide = robotPose.getY() >= Constants.FIELD_WIDTH_Y / 2.0;

    if (alliance == Alliance.Blue) {
      return isLeftSide ? Constants.blueLeftShuttlePosition : Constants.blueRightShuttlePosition;
    }
    return isLeftSide ? Constants.redLeftShuttlePosition : Constants.redRightShuttlePosition;
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
   */
  private double computeAutoYawVelocityRadiansPerSecond() {
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
  private double computeDistanceToPredictedTarget() {
    Pose2d botPos = drivetrain.getRobotPosition();
    return shooterTranslationFromRobotPose(botPos).getDistance(predictedTarget);
  }

  /**
   * Compute the predicted target accounting for velocities and time of flight.
   *
   * @return the predicted target
   */
  protected Translation2d computePredictedTarget() {
    if (desiredTarget == null) {
      return null;
    }

    // compute robot-relative field velocity
    ChassisSpeeds fieldSpeeds = drivetrain.getChassisSpeedsFieldCentric();
    Translation2d fieldSpeedsTranslation = new Translation2d(fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond);
    if (doCompensateForRotation) {
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
      fieldSpeedsTranslation = fieldSpeedsTranslation.plus(rotVelocityVector);
    }

    // NOTE: This function is called before the timeOfFlight is updated, 
    // It uses the timeOfFlight from the last tick because otherwise, 
    // this would be a circular dependency. 
    Translation2d predicted = desiredTarget.minus(fieldSpeedsTranslation.times(predictedTimeOfFlight));
    return predicted;
  }
  
  /**
   * 
   * @return desired turret rotation2d w.r.t. the robot for the turret to face the target
   * NOTE: this is a different angle than the robot would use for autoYaw. Since the 
   * shooter is offset from the center of the robot. 
   * Returns the rotation relative to the robot accounting for the current robot yaw.
   */
  protected Rotation2d computeTurretAngle() {
    Pose2d botPos = drivetrain.getRobotPosition();
    Translation2d shooterTranslation = shooterTranslationFromRobotPose(botPos);
    double dx = predictedTarget.getX() - shooterTranslation.getX();
    double dy = predictedTarget.getY() - shooterTranslation.getY();

    double angleRad = Math.atan2(dy, dx);
    // turret zero is defined same as robot yaw, whereas botPose seems to be defined 
    // 180 from robot yaw, so need to add
    // 180 here to be consistent with robot yaw.
    // return Rotation2d.fromRadians(angleRad).minus(botPos.getRotation()).plus(Rotation2d.fromDegrees(180.0));
    return Rotation2d.fromRadians(angleRad).minus(botPos.getRotation());
  }

  /**
   * 
   * @return desired field relative Rotation2d for the robot to face the target
   *         accounts for shooterYawOffset relative to robot
   */
  protected Rotation2d computeYawToFaceTarget() {
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
  protected double hoodExtensionForDistance(double distanceMeters) {
    double ext = hoodExtensionLookupTable.queryTable(distanceMeters);
    return MathUtil.clamp(ext, Constants.hoodMinPositionRotations, Constants.hoodMaxPositionRotations);
  }

  /**
   * helper to get the kicker RPM for a given distance
   * 
   * @param distanceMeters
   * @return kicker RPM
   */
  private double kickerRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(kickerRpmLookupTable.queryTable(distanceMeters), Constants.KICKER_MIN_RPM,
        Constants.KICKER_MAX_RPM);
  }

  /**
   * helper to get the shooter pose from the robot pose in field coordinates
   * @param robotPose in field coordinates
   * @return shooter pose in field coordinates
   */
  protected Translation2d shooterTranslationFromRobotPose(Pose2d robotPose) {
    return robotPose.getTranslation().plus(
        Constants.shooterOffsetFromCenterOfRobot.rotateBy(robotPose.getRotation()));
  }

  /**
   * helper to get the shooter RPM for a given distance
   * 
   * @param distanceMeters
   * @return shooter RPM
   */
  protected double shooterRpmForDistance(double distanceMeters) {
    return MathUtil.clamp(shooterRpmLookupTable.queryTable(distanceMeters), Constants.SHOOTER_MIN_RPM,
        Constants.SHOOTER_MAX_RPM);
  }

  private double tofForDistance(double distanceMeters) {
    return tofLookupTable.queryTable(distanceMeters);
  }

  private Rotation2d getCurrentYawRotation() {
    if (InstalledHardware.useTurretForAiming && subsystemCollection.getTurretSubsystem() != null) {
      return drivetrain.getGyroscopeRotation().plus(Rotation2d.fromRadians(subsystemCollection.getTurretSubsystem().getAngleRadians()));
    }
    return drivetrain.getGyroscopeRotation();
  }
}