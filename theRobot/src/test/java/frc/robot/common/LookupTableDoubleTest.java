package frc.robot.common;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import frc.robot.common.LookupTableDouble;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;


class LookupTableDoubleTest {
  static final double DELTA = 1e-2; // acceptable deviation range
  
  @BeforeEach // this method will run before each test
  void setup() {
  }

  @AfterEach // this method will run after each test
  void shutdown() throws Exception {
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
}

