// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: HardwareConstants.java
// Intent: Forms key HARDWARE-only constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

public class HardwareConstants {

    // *******************************************************************
    // CTRE motor constants
    public static final double ctreStatorCurrentMaximumAmps = 100.0;
    public static final double ctreSupplyCurrentMaximumAmps = 50.0;
    public static final double ctreSupplyVoltageTimeConstant = 0.02;

    // ********************************************************************
    // Shooter Constants
    public static final int shooterSmartCurrentLimitAmps = 40;
    public static final double nominalVoltageCompensationVolts = 12.0;
    
    // ********************************************************************
    // Servo Constants
    public static final int HOOD_MIN_EXT = 1000;
    public static final int HOOD_MAX_EXT = 2000;
    // ********************************************************************
    // CAN Optimization Constants
    // ctre
    public static final int ctreMotorStatusFramePeriodMilliseconds = 20; // 20ms refresh rate
    public static final int ctreSensorStatusFramePeriodMilliseconds = 30; // 30ms refresh rate
    public static final double ctreMotorStatusFramePeriodFrequencyHertz = 1000.0
            / (double) HardwareConstants.ctreMotorStatusFramePeriodMilliseconds;
    public static final double ctreSensorStatusFramePeriodFrequencyHertz = 1000.0
            / (double) HardwareConstants.ctreSensorStatusFramePeriodMilliseconds;
    // playing with fusion
    public static final double playingWithFusionSensorPeriodMilliseconds = 24; // 24ms refresh rate as specified in
                                                                               // their documentation (max refresh rate)
}
