// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: Constants.java
// Intent: Place for constants
// ************************************************************
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int talonCanID = 4;

  // most of the distance sensors have this max range. 
  public static final double MAX_RANGE_INCHES = 78.74; // 2 meter conversion
  public static final double maxRangeLaserCanInches = 157.48; // 4 meters in inches

  // CAN Ids for sensors and spinner
  public static final int rangeSensorCTRECanID = 32;
  public static final int rangeSensorPWFCanID = 33;
  public static final int rangeSensorLaserCanID = 34;
  public static final int SPINNER_CAN_ID = 6;

  // AMPS and Voltage constants for motor config 
  public static final double motorSupplyCurrentMaximumAmps = 50.0;
  public static final double motorSupplyVoltageTimeConstant = 0.2;
  public static final double falconMaxVoltage = 12.0;
  public static final double motorStatorCurrentMaximumAmps = 100.0;

  // Spinner motor speed constant
  public static final double motorSpeed = 10.0;


  public static final int kDriverControllerPort = 0;
  
}
