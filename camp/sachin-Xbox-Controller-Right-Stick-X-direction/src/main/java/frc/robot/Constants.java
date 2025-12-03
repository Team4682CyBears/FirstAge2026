// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: Constants.java
// Intent: Robot-wide constant definitions
// ************************************************************

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be instantiated by any
 * other class. If you add constants to each subsystem, you should also add the
 * constants' names to the log.
 */
public final class Constants {
  
  /**
   * Constants related to operator input devices.
   */
  public static class OperatorConstants {
    /** Port for the driver's Xbox controller */
    public static final int kDriverControllerPort = 0;
    
    /** CAN ID for the TalonFX motor */
    public static final int talonFXCANID = 4;
  }

  private Constants() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}