package frc.robot.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)), new Rotation2d(0.0)));
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)), shooterAimer.getYawToFaceTarget()));
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
    // start 1m from target in x
    // have to set the drivetrain x/y first, and then apply the auto yaw (since the PID that would set it doesn't work in sim)
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)), new Rotation2d(0.0)));
    // make position 5 degrees off and see what the PID speed is
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)), 
        shooterAimer.getYawToFaceTarget().plus(Rotation2d.fromDegrees(20))));
    double yawVelocityRadiansPerSecond = shooterAimer.computeAutoYawVelocityRadiansPerSecond();
    System.out.println("auto yaw rotational velocity " + yawVelocityRadiansPerSecond);
    double translationalVelocity = yawVelocityRadiansPerSecond * Constants.shooterOffsetFromCenterOfRobot.getNorm();
    System.out.println("auto yaw translations velocity (mps)" + translationalVelocity);
    Translation2d translationalVelocityRobotCentric = Constants.shooterOffsetFromCenterOfRobot.times(yawVelocityRadiansPerSecond).rotateBy(Rotation2d.fromDegrees(90));
    System.out.println("auto yaw translations velocity robot centric (mps)" + translationalVelocityRobotCentric);
    Translation2d translationalVelocityFieldCentric = translationalVelocityRobotCentric.rotateBy(drivetrain.getGyroscopeRotation());
    System.out.println("auto yaw translation velocity field centric (mps)" + translationalVelocityFieldCentric);
    Translation2d translationalError = translationalVelocityFieldCentric.times(Constants.PROJECTILE_TIME_OF_FLIGHT_SECONDS);
    System.out.println("translation error (m) " + translationalError);

  }

}
