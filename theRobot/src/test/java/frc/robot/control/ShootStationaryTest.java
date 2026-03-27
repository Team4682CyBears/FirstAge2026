// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShootStationaryTest.java
// Intent: a class to test ShooterAimer.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.DrivetrainSubsystem;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ShootStationaryTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  SubsystemCollection subsystemCollection;
  DrivetrainSubsystem drivetrain;
  ShooterAimer shooterAimer;

  @BeforeEach // this method will run before each test
  void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    subsystemCollection = new SubsystemCollection();
    drivetrain = new DrivetrainSubsystem(subsystemCollection);
    shooterAimer = new ShooterAimer(drivetrain, subsystemCollection);
    shooterAimer.calculate();
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    // need to close drivetrain and shooterAimer here...
  }

  /*
   * Static shooting tests
   */

  @Test
  void hoodExtensionValidValue() {
    // known lookup table value
    double input = 3.4408;
    double output = 0.25;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void hoodExtensionInterpolatedValue() {
    // interpolated lookup table value
    // { 3.4408, 0.25 },
    // { 4.7448, 0.38 },
    double input = 4.0;
    double numerator = (0.25 * (4.7448 - input)) + (0.38 * (input - 3.4408));
    double denominator = (4.7448 - 3.4408);
    double output = numerator / denominator;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void hoodExtensionClampBelowMin() {
    // clamp to min when input below min
    double input = 0.5; // below min
    double output = 0.0;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void hoodExtensionClampAboveMax() {
    // clamp to max when input above max
    double input = 9.0; // below min
    double output = 0.50;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void shooterRPMValidValue() {
    // known lookup table value
    double input = 1.3037;
    double output = 3000;
    double hoodPosition = shooterAimer.shooterRpmForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void shooterRPMInterpolatedValue() {
    // interpolated lookup table value
    // { 1.3037, 3000 },
    // { 4.7448, 4200 },
    double input = 4.0;
    double numerator = (3000 * (4.7448 - input)) + (4200 * (input - 1.3037));
    double denominator = (4.7448 - 1.3037);
    double output = numerator / denominator;
    double hoodPosition = shooterAimer.shooterRpmForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void shooterRPMClampBelowMin() {
    // clamp to min when input below min
    double input = 0.5; // below min
    double output = 2912;
    double hoodPosition = shooterAimer.shooterRpmForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void ShooterRPMClampAboveMax() {
    // clamp to max when input above max
    double input = 9.0; // above max
    double output = 5000;
    double hoodPosition = shooterAimer.shooterRpmForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void RobotYawTargetDeadOn() {
    // robot yaw should be 180 when dead on to target
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)), new Rotation2d(0.0)));
    assertEquals(Rotation2d.fromDegrees(180.0).plus(Constants.shooterYawOffset).getDegrees(),
        shooterAimer.computeYawToFaceTarget().getDegrees(), DELTA);
  }

  @Test
  void RobotYawTarget45degrees() {
    // robot yaw should be -135 (180+45 + shooterYawOffset) degrees when offset
    // equal amounts in x/y
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 1.0)), new Rotation2d(0.0)));
    assertEquals(Rotation2d.fromDegrees(180.0 + 45.0).plus(Constants.shooterYawOffset).getDegrees(),
        shooterAimer.computeYawToFaceTarget().getDegrees(), DELTA);
  }

  @Test
  void RobotYawTarget90degrees() {
    // robot yaw should be -90 when to the side of target
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(0.0, 1.0)), new Rotation2d(0.0)));
    assertEquals(Rotation2d.fromDegrees(-90.0).plus(Constants.shooterYawOffset).getDegrees(),
        shooterAimer.computeYawToFaceTarget().getDegrees(),
        DELTA);
  }

  @Test
  void TurretYawTargetDeadOn() {
    // turret yaw should be 0 when dead on to target
    // to make the turret dead on target, need to shift by shooter offset
    drivetrain
        .setRobotPosition(new Pose2d(
            Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)).minus(Constants.shooterOffsetFromCenterOfRobot),
            new Rotation2d(0.0)));
    // TODO put this test back once odometry bug is fixed
    // assertEquals(0.0,
    // shooterAimer.computeTurretAngle().getDegrees(), DELTA);
  }

  @Test
  void TurretYawTargetNotDeadOn() {
    // turret yaw should be slightly (counterclockwise) positive when robot is dead
    // on to target
    // turret is shifted by shooter offset w.r.t. the robot, so restulting in a
    // small positive angle
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)), new Rotation2d(0.0)));
    double expectedAngleRadians = -Math.atan2(Constants.shooterYOffsetFromCenterOfRobot,
        1.0 + Constants.shooterXOffsetFromCenterOfRobot);
    double actualAngleDegrees = shooterAimer.computeTurretAngle().getDegrees();
    // TODO put this test back once odometry bug is fixed
    // assertEquals(Units.radiansToDegrees(expectedAngleRadians),
    // actualAngleDegrees, DELTA);
    // assertTrue(actualAngleDegrees > 0.0);
  }

  @Test
  void TurretYawTargetWithRobotYaw() {
    // when shooter is 1m left and 1m back of target, and robot rotated CCW 90
    // (positive 90)
    // turret yaw should be (counterclockwise) positive 225 (=180+45) degrees
    // turret is offset from robot by a set offset.
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(
            new Translation2d(
                1.0 + Constants.shooterXOffsetFromCenterOfRobot,
                1.0 - Constants.shooterYOffsetFromCenterOfRobot)),
            Rotation2d.fromDegrees(90.0)));
    // get the actual in degrees [0..360]
    double actualAngleDegrees = (shooterAimer.computeTurretAngle().getDegrees() + 360) % 360;
    // TODO put this test back once odometry bug is fixed
    // assertEquals(225.0, actualAngleDegrees, DELTA);
  }

}
