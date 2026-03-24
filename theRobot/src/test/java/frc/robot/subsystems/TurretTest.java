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
  
  @BeforeEach // this method will run before each test
  void setup() {
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
  }

  @Test
  void simpleSetThenGet(){
    // sets and gets value in turret's valid range
    TurretSubsystem turret = new TurretSubsystem(Constants.turretMotorCanId);
    double targetRadians = Units.degreesToRadians(30.0);
    turret.setTargetAngleRadians(targetRadians);
    double error = MathUtil.angleModulus(targetRadians - turret.getAngleRotation2d().getRadians());
    assertEquals(error, 0.0, DELTA);
  }

}