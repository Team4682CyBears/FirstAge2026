// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: Constants.java
// Intent: Robot-wide constant definitions
// ************************************************************

package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be instantiated by any
 * other class. If you add constants to each subsystem, you should also add the
 * constants' names to the log.
 */
public final class Constants {
  public static final int kDriverControllerPort = 0;
  public static final int talonFXCANID = 4;

  public static final double motorSupplyCurrentMaximumAmps = 50.02;
  public static final double motorSupplyVoltageTimeConstant = 0.02;

  public static final double falconMaxVoltage = 12.0;
  public static final double motorStatorCurrentMaximumAmps = 100.0;
}