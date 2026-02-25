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

  // most of the distance sensors have this max range. 
  public static final double maxRangeLaserCanInches = 2; // 4 meters in inches low of non cardboard and high of carboard

  //TODO IF WE ARE USING OTHER SENSORS ACOUNT FOR OFFSET IN IS VALID CHECK

  // CAN Ids for sensors and spinner
  public static final int rangeSensorCTRECanID = 32;
  public static final int rangeSensorPWFCanID = 33;
  public static final int rangeSensorLaserCanID = 34;
  public static final int SPINNER_CAN_ID = 8;

  // AMPS and Voltage constants for motor config 
  public static final double motorSupplyCurrentMaximumAmps = 50.0;
  public static final double motorSupplyVoltageTimeConstant = 0.2;
  public static final double falconMaxVoltage = 12.0;
  public static final double motorStatorCurrentMaximumAmps = 100.0;

  // Spinner motor speed constant measured in rpm
  public static double motorSpeed = 150; //275 10 balls per second / 4 balls per rotation
  
  public static final int kickerTalonCanId = 4;
  public static final double kickerMotorGearRatio = 1.0; // 1:1 for now, but may change if we add a gearbox
  public static final int kDriverControllerPort = 0;
}
