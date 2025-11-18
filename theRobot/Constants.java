// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
  public static int reefTofSensorCanID = 14;
  public static int reefTofSensorCanID2 = 17;
  public static double tofRangeInches = 16.0;
  //Motor Config constant variables
  public static final int kDriverControllerPort = 0;
  public static final double motorStatorCurrentMaximumAmps = 100.0;
  public static final double motorSupplyCurrentMaximumAmps = 50.0;
  public static final double motorSupplyVoltageTimeConstant = 0.02;
  //Motor Talon Port
  public static final int motorCanID = 3;
  public static final double falconMaxVoltage = 12.0;
  public static final int falconFreeSpeedRps = 6380/60;
  public static final double minimumMotorSpeedRpm = 0.25*60;

}
