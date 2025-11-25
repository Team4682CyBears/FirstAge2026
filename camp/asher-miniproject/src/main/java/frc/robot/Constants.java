// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: Constants.java
// Intent: Forms key constants required for this robot
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 


package frc.robot;

import com.ctre.phoenix6.signals.SensorDirectionValue;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int motorCanID = 3;
  public static final int encoderCanID = 4;
  public static final double motorSupplyCurrentMaximumAmps = Constants.ctreSupplyCurrentMaximumAmps;
  public static final double motorSupplyVoltageTimeConstant = Constants.ctreSupplyVoltageTimeConstant;
  public static final double ctreSupplyCurrentMaximumAmps = 50.0;
  public static final double ctreSupplyVoltageTimeConstant = 0.02;

  public static final double falconMaxVoltage = 12.0;
  public static final double motorStatorCurrentMaximumAmps = 100.0;

  public static final SensorDirectionValue shooterAngleSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
  public static final double encoderAbsoluteOffsetDegrees = 0;
  public static final double absoluteSensorDiscontinuityPoint = 0.5;
}
