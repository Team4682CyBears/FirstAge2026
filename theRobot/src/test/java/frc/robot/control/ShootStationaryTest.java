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
    // set target
    shooterAimer.setDesiredTarget(Constants.blueHubPosition);
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
    double output = 0.45;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void hoodExtensionInterpolatedValue() {
    // interpolated lookup table value
    // 3.4408, 0.45 },
    // { 4.7448, 0.637
    double input = 4.0;
    double numerator = (0.45 * (4.7448 - input)) + (0.637 * (input - 3.4408));
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
    double output = Constants.hoodMinPositionRotations;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void hoodExtensionClampAboveMax() {
    // clamp to max when input above max
    double input = 9.0; // below min
    double output = Constants.hoodMaxPositionRotations;
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
    // { 4.7448, 4000 },
    double input = 4.0;
    double numerator = (3000 * (4.7448 - input)) + (4000 * (input - 1.3037));
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
    double input = 9.0; // below min
    double output = 6500;
    double hoodPosition = shooterAimer.shooterRpmForDistance(input);
    assertEquals(
        output, hoodPosition, DELTA);
  }

  @Test
  void RobotYawTargetDeadOn() {
    // robot yaw should be 180 when dead on to target
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 0.0)), new Rotation2d(0.0)));
    assertEquals(Rotation2d.fromDegrees(180.0).plus(Constants.shooterYawOffset).getDegrees(), shooterAimer.getYawToFaceTarget().getDegrees());
  }

  @Test
  void RobotYawTarget45degrees() {
    // robot yaw should be -135 (-180+45 - shooterYawOffset) degrees when offset equal amounts in x/y
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(1.0, 1.0)), new Rotation2d(0.0)));
    assertEquals(Rotation2d.fromDegrees(-180.0 + 45.0).plus(Constants.shooterYawOffset).getDegrees(), shooterAimer.getYawToFaceTarget().getDegrees());
  }

  @Test
  void RobotYawTarget90degrees() {
    // robot yaw should be -90 when to the side of target
    drivetrain
        .setRobotPosition(new Pose2d(Constants.blueHubPosition.plus(new Translation2d(0.0, 1.0)), new Rotation2d(0.0)));
    assertEquals(Rotation2d.fromDegrees(-90.0).plus(Constants.shooterYawOffset).getDegrees(), shooterAimer.getYawToFaceTarget().getDegrees());
  }
}
