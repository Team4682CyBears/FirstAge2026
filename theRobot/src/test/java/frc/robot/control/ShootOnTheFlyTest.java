package frc.robot.control;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.common.LookupTableDouble;

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
    drivetrain = new DrivetrainSubsystem(subsystemCollection);
    shooterAimer = new ShooterAimer(drivetrain, subsystemCollection);
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
    // need to close drivetrain and shooterAimer here...
  }

  @Test
  void singleValueLUTGreaterThanMaxReturnsDefault(){
    // a LUT with a single entry returns default when query value is greater 
    double[][] lookupTable = {{30, 200}};
    LookupTableDouble LUT = new LookupTableDouble(lookupTable, 1000.0); 
    assertEquals(
      1000.0, LUT.queryTable(40.0)
    );
  }

  @Test
  void LUTGreaterThanMaxReturnsClamp(){
    // a LUT returns max output when query value is greater 
    double[][] lookupTable = {{30, 200}, {35, 300}};
    LookupTableDouble LUT = new LookupTableDouble(lookupTable); 
    assertEquals(
      300.0, LUT.queryTable(40.0)
    );
  }

  @Test
  void singleValueLUTLessThanMinReturnsDefault(){
    // a LUT with a single entry returns default value when query value is lower
    double[][] lookupTable = {{30, 200}};
    LookupTableDouble LUT = new LookupTableDouble(lookupTable, 1000.0); 
    assertEquals(
      1000.0, LUT.queryTable(20.0)
    );
  }

  @Test
  void LUTLessThanMinReturnsClamp(){
    // a LUT returns min output value when query value is lower
    double[][] lookupTable = {{30, 200}, {35, 300}};
    LookupTableDouble LUT = new LookupTableDouble(lookupTable); 
    assertEquals(
      200.0, LUT.queryTable(20.0)
    );
  }

  @Test
  void singleValueLUTInBoundsReturnsValue(){
    // a LUT with a single entry returns output value when query matches
    double[][] lookupTable = {{30, 200}};
    LookupTableDouble LUT = new LookupTableDouble(lookupTable, 1000.0); 
    assertEquals(
      200.0, LUT.queryTable(30.0)
    );
  }

  @Test
  void LUTInBoundsReturnsValue(){
    // a LUT returns interpolated output value when query in bounds
    double[][] lookupTable = {{30, 200}, {36, 300}};
    LookupTableDouble LUT = new LookupTableDouble(lookupTable); 
    assertEquals(
      250.0, LUT.queryTable(33.0)
    );
  }

  @Test 
  void hoodExtensionValidValue() {
    // known lookup table value
    double input = 3.4408;
    double output = 0.45;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output , hoodPosition, DELTA); 
  }

  @Test 
  void hoodExtensionInterpolatedValue() {
    // interpolated lookup table value
    // 3.4408, 0.45 },
    //  { 4.7448, 0.637 
    double input = 4.0;
    double numerator = (0.45 * (4.7448- input)) + (0.637 * (input - 3.4408));
    double denominator = (4.7448 - 3.4408);
    double output = numerator / denominator;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
         output, hoodPosition, DELTA); 
  }

  @Test 
  void hoodExtensionDefaultBelowMin() {
    // known lookup table value
    double input = 0.5; // below min
    double output = Constants.hoodMinPositionRotations;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output , hoodPosition, DELTA); 
  }

  @Test 
  void hoodExtensionDefaultAboveMax() {
    // known lookup table value
    double input = 9.0; // below min
    double output = Constants.hoodMaxPositionRotations;
    double hoodPosition = shooterAimer.hoodExtensionForDistance(input);
    assertEquals(
        output , hoodPosition, DELTA); 
  }
}

