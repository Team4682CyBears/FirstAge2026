// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: TurretTest.java
// Intent: a class to test TurretSubsystem.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.control.Constants;


class TurretTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  // allocate turret here so it can be closed by the shutdown method
  TurretSubsystem turret = null;
  
  @BeforeEach // this method will run before each test
  void setup() {
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    turret.close();
  }

  @Test
  void simpleSetThenGet(){
    // sets and gets value in turret's valid range
    turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = Units.degreesToRadians(30.0);
    turret.setTargetAngleRadians(targetRadians);
    double error = MathUtil.angleModulus(targetRadians - turret.getAngleRadians());
    assertEquals(error, 0.0, DELTA);
  }

  @Test
  void SmallNegativeAngleClampsToMin(){
    // sets and gets small negative value
    turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = Units.degreesToRadians(-1.0);
    turret.setTargetAngleRadians(targetRadians);
    assertEquals(Units.degreesToRadians(Constants.turretMinAngleDegrees), turret.getAdjustedTurretMechanismPositionRadians(), DELTA);
  }

  @Test
  void LargePositiveAngleClampsToMin(){
    // sets and gets large positive value
    turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = Units.degreesToRadians(359.0);
    turret.setTargetAngleRadians(targetRadians);
    assertEquals(Units.degreesToRadians(Constants.turretMinAngleDegrees), turret.getAdjustedTurretMechanismPositionRadians(), DELTA);
  }

  @Test
  void SmallNegativeAngleClampsToMax(){
    // sets and gets small negative value
    turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = Units.degreesToRadians(-4.0);
    turret.setTargetAngleRadians(targetRadians);
    assertEquals(Units.degreesToRadians(Constants.turretMaxAngleDegrees), turret.getAdjustedTurretMechanismPositionRadians(), DELTA);
  }

  @Test
  void LargePositiveAngleClampsToMax(){
    // sets and gets large positive value
    turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = Units.degreesToRadians(356.0);
    turret.setTargetAngleRadians(targetRadians);
    assertEquals(Units.degreesToRadians(Constants.turretMaxAngleDegrees), turret.getAdjustedTurretMechanismPositionRadians(), DELTA);
  }

  @Test
  void LargePositiveAngleWraps(){
    // sets and gets large positive value
    turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = 2 * Math.PI + 0.1;
    turret.setTargetAngleRadians(targetRadians);
    assertEquals(0.1, turret.getAdjustedTurretMechanismPositionRadians(), DELTA);
  }

  @Test
  void LargeNegativeAngleWraps(){
    // sets and gets large negative value
    turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = -2 * Math.PI + 1.0;
    turret.setTargetAngleRadians(targetRadians);
    assertEquals(1.0, turret.getAdjustedTurretMechanismPositionRadians(), DELTA);
  }
}