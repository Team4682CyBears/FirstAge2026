package frc.robot.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.lang.constant.Constable;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.DrivetrainSubsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShootOnTheFlyTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  SubsystemCollection subsystemCollection;
  DrivetrainSubsystem drivetrain;
  ShooterAimer shooterAimer;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    subsystemCollection = new SubsystemCollection();
    subsystemCollection.setCameraSubsystem(null);
    drivetrain = new DrivetrainSubsystem(subsystemCollection);
    shooterAimer = new ShooterAimer(drivetrain, subsystemCollection);
    // enable driver station and set alliance
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // delay 100ms to allow enable to take effect
    Timer.delay(0.100);

    // set target
    shooterAimer.setDesiredTarget(shooterAimer.getHubPositionFromAlliance());
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    // need to close drivetrain and shooterAimer here...
  }

  @Test
  void RobotYawTargetDeadOnFadeShot() {
    // robot yaw should be 180 when dead on to target
    // target should move farther when driving away
    
    // setup drivetrain
    // start 1m from target in x
    // have to set the drivetrain x/y first, and then apply the auto yaw (since the PID that would set it doesn't work in sim)
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(-1.0, 0.0)), new Rotation2d(0.0)));
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(-1.0, 0.0)), shooterAimer.getYawToFaceTarget()));
    // drive 1 mps away from target in x
    double vx = -1.0;
    double vy = 0.0;
    drivetrain.driveFieldCentricShooting(new ChassisSpeeds(vx, vy, 0.0));

    // compute expected target
    Translation2d expectedTarget = Constants.blueHubPosition.plus(
      new Translation2d(-vx * Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS, -vy * Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS));

    System.out.println("original target " + Constants.blueHubPosition);
    System.out.println("expected target " + expectedTarget);
    System.out.println("actual target " + shooterAimer.computePredictedTarget());
    assertTrue(expectedTarget.equals(shooterAimer.computePredictedTarget()));
  }

  @Test
  void RobotYawRotationalVelocityMatters() {
    // setup drivetrain
    // start to the left of the target and move backwards
    // have to set the drivetrain x/y first, and then apply the auto yaw (since the PID that would set it doesn't work in sim)
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(-1.0, 2.0)), new Rotation2d(0.0)));
    // make position 5 degrees off and see what the PID speed is
    Rotation2d gyroscopeRotation = shooterAimer.getYawToFaceTarget();
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(-1.0, 2.0)), 
        gyroscopeRotation.plus(Rotation2d.fromDegrees(5.0))));
    double yawVelocityRadiansPerSecond = shooterAimer.computeAutoYawVelocityRadiansPerSecond();
    System.out.println("auto yaw rotational velocity " + yawVelocityRadiansPerSecond);
    // drive 1 mps away from target in x
    double vx = -1.0;
    double vy = 0.0;
    drivetrain.driveFieldCentricShooting(new ChassisSpeeds(vx, vy, yawVelocityRadiansPerSecond));

    // compute expected target
    Translation2d expectedTargetPreRotation = Constants.blueHubPosition.plus(
      new Translation2d(-vx * Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS, -vy * Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS));

    // construct expected rot vector
    Translation2d rotVector = new Translation2d(
      Constants.shooterOffsetFromCenterOfRobot.getNorm(),
      gyroscopeRotation).times(Math.abs(yawVelocityRadiansPerSecond));

    Translation2d expectedTargetPostRotation = expectedTargetPreRotation.minus(
      rotVector.times(Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS));

    // compute actual target
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(vx, vy, yawVelocityRadiansPerSecond);
    Translation2d fieldSpeedsTranslation = new Translation2d(fieldSpeeds.vxMetersPerSecond,
        fieldSpeeds.vyMetersPerSecond);
      // TODO test this on-robot. Not sure about direction of the velocity. Might be -90?
      // velocity due to rotation = angular velocity * shooterOffset rotated by 90
      // degrees (it's orthogonal to the radius).
      System.out.println("field speeds " + fieldSpeeds);
      Translation2d rotationalVelocityRobotCentric = Constants.shooterOffsetFromCenterOfRobot
          .times(fieldSpeeds.omegaRadiansPerSecond).rotateBy(Rotation2d.fromDegrees(90));
      Translation2d rotationalVelocityFieldCentric = rotationalVelocityRobotCentric
          .rotateBy(gyroscopeRotation);
      fieldSpeedsTranslation = fieldSpeedsTranslation.plus(rotationalVelocityFieldCentric);
      Translation2d rotVecActual = fieldSpeedsTranslation.times(Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS);

    System.out.println("original target " + Constants.blueHubPosition);
    System.out.println("expected target pre rotation" + expectedTargetPreRotation);
    System.out.println("rotationVector " + rotVector);
    System.out.println("expected target post rotation" + expectedTargetPostRotation);
    System.out.println("Actual rot vec correction" + rotVecActual);
    //System.out.println("actual target " + shooterAimer.computePredictedTarget());
    assertTrue(expectedTargetPostRotation.equals(shooterAimer.computePredictedTarget()));
  }

  @Test
  void ErrorCalcs() {
        // setup drivetrain
    // start 1m from target in x
    // have to set the drivetrain x/y first, and then apply the auto yaw (since the PID that would set it doesn't work in sim)
    double xoffset = -3.0;
    double yoffset = 0.0;
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(xoffset, yoffset)), new Rotation2d(0.0)));
    // make position x degrees off and see what the PID speed is
    double rotOffset = 0.0;
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(xoffset, yoffset)), 
        shooterAimer.getYawToFaceTarget().plus(Rotation2d.fromDegrees(rotOffset))));
    // drive 1 mps away from target in x
    double vx = 0.0;
    double vy = 0.0;
    drivetrain.driveFieldCentricShooting(new ChassisSpeeds(vx, vy, 0.0));
    double distance = shooterAimer.getDistanceToPredictedTarget();
    System.out.println("Distance " + distance);
    System.out.println("Angle " + shooterAimer.hoodExtensionForDistance(distance));
    System.out.println("V Shooter " + shooterAimer.shooterRpmForDistance(distance));
    System.out.println("Actual TOF " + shooterAimer.tofForDisatnce(distance));
  }

}
