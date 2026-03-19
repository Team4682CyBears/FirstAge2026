// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShootOnTheFlyTest.java
// Intent: a class to test ShooterAimer.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
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
  drivetrain.setShooterAimer(shooterAimer);
    // enable driver station and set alliance
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    // delay 100ms to allow enable to take effect
    Timer.delay(0.100);

    // set target
    shooterAimer.setDesiredTarget(shooterAimer.getHubPositionFromAlliance());
    drivetrain.setSwerveYawMode(SwerveYawMode.AUTO);
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    // need to close drivetrain and shooterAimer here...
  }

  @Test
  void RobotYawTargetDeadOnFadeShot() {
    // robot yaw should be 180 when dead on to target
    // target should move farther when driving away

    // turn off rotation compensation for this test
    shooterAimer.doCompensateForRotation = false;

    // setup drivetrain
    // start 1m from target in x
    // have to set the drivetrain x/y first, and then apply the auto yaw (since the
    // PID that would set it doesn't work in sim)
    drivetrain
        .setRobotPosition(
            new Pose2d(Constants.blueHubPosition.plus(new Translation2d(-1.0, 0.0)), Constants.shooterYawOffset));
    shooterAimer.calculate();
    // drive 1 mps away from target in x
    double vx = -1.0;
    double vy = 0.0;
    drivetrain.driveFieldCentricShooting(new ChassisSpeeds(vx, vy, 0.0));
    drivetrain.periodic();

    double tof = shooterAimer.getTimeOfFlight();
    // compute expected target
    Translation2d expectedTarget = Constants.blueHubPosition.plus(
        new Translation2d(-vx * tof, -vy * tof));

    Translation2d actualTarget = shooterAimer.computePredictedTarget(); 
    //TODO debug why this is different than getPredictedTarget
    System.out.println("original target " + Constants.blueHubPosition);
    System.out.println("expected target " + expectedTarget);
    System.out.println("drivetrain position " + drivetrain.getRobotPosition());
    System.out.println("drivetrain speeds " + drivetrain.getChassisSpeedsFieldCentric());
    System.out.println("actual target " + actualTarget);
    assertTrue(expectedTarget.getDistance(actualTarget) <= DELTA);
  }

  @Test
  void RobotYawRotationalVelocityMatters() {
    // override to enable shooter displayDiagnostics for debugging
    shooterAimer.displayDiagnostics = true;
    // setup drivetrain
    // start to the left of the target and move backwards
    // have to set the drivetrain x/y first, and then apply the auto yaw (since the
    // PID that would set it doesn't work in sim)
    // make position 5 degrees off to generate some autoYaw velocity
    Rotation2d robotYaw = Rotation2d.fromDegrees(-123.43 + 5);
    drivetrain
        .setRobotPosition(
            new Pose2d(Constants.blueHubPosition.plus(new Translation2d(-1.0, 2.0)), robotYaw));
    // drive 1 mps away from target in x
    double vx = -1.0;
    double vy = 0.0;
    drivetrain.driveFieldCentricShooting(new ChassisSpeeds(vx, vy, 0.0));
    // run periodic once to pick up starting conditions and then run again to 
    // recalculate iteratives (like calculated target and distance)
    drivetrain.periodic();    
    // this is the yaw velocity that will be used in the next predicted target
    double yawVelocityRadiansPerSecond = shooterAimer.getAutoYawVelocityRadiansPerSecond();
    System.out.println("auto yaw rotational velocity " + yawVelocityRadiansPerSecond);
    drivetrain.periodic();

    // compute expected target
    double tof = shooterAimer.getTimeOfFlight();
    Translation2d expectedTargetPreRotation = Constants.blueHubPosition.plus(
        new Translation2d(-vx * tof,
            -vy * tof));

    // construct expected rot vector
    Translation2d rotVector = new Translation2d(
        // magnitude is the shooter offset from center of robot * the rotational
        // velocity
        Constants.shooterOffsetFromCenterOfRobot.getNorm() * Math.abs(yawVelocityRadiansPerSecond),
        // angle is orthogonal to the vector from robot center to the hub
        // direction depends on the rotation direction
        robotYaw
            .minus(Constants.shooterYawOffset)
            .minus(Rotation2d.fromDegrees(Math.copySign(-90, yawVelocityRadiansPerSecond))));

    Translation2d expectedTargetPostRotation = expectedTargetPreRotation.minus(
        rotVector.times(shooterAimer.getTimeOfFlight()));

    Translation2d predictedTarget = shooterAimer.getPredictedTarget();

    System.out.println("original target " + Constants.blueHubPosition);
    System.out.println("expected target pre rotation" + expectedTargetPreRotation);
    System.out.println("rotationVector " + rotVector);
    System.out.println("expected target post rotation" + expectedTargetPostRotation);
    System.out.println("actual target " + predictedTarget);
    // TODO find the small mismatch. Why is it not matching exactly??
    assertTrue(expectedTargetPostRotation.getDistance(predictedTarget) <= 2 * DELTA);
  }
}
