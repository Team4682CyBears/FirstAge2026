// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: InstalledHardware.java
// Intent: Forms a listing of switches that will help to debug code better as hardware is available (or not available).
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

/**
 * A class devoted to installed hardware constants. Use this to decide if
 * hardware is enabled on the robot or not.
 * All developers to use this to protect their subsystems and commands from
 * using hardware that is not actually installed
 * on the robot at hand. Used to assist in development stages and make it easier
 * to quickly remove a piece of hardware
 * from the robot.
 */
public class InstalledHardware {
    // Basic Hardware
    public static boolean powerDistributionPanelInstalled = true;

    // Onboard Hardware - Orentation/Navigation Hardware
    public static boolean wifiRadioInstalled = true;
    public static boolean limelightInstalled = true;

    // External Input Hardware
    public static boolean driverXboxControllerInstalled = true;
    public static boolean coDriverXboxControllerInstalled = true;

    // DriveTrain Related Hardware
    public static boolean drivetrainInstalled = true;
    public static boolean bareDrivetrainInstalled = true;

    // Shooter related hardware
    public static boolean shooterInstalled = true;
    public static boolean hoodMotorInstalled = true;
    public static boolean hoodEncoderInstalled = true;

    // Spindexer
    public static boolean spindexerInstalled = true;
    public static boolean spindexerSensorInstalled = false;
    
    // Kicker
    public static boolean kickerInstalled = true;

    // Intake wrist encoder
    public static boolean intakeWristEncoderInstalled = true;
    public static boolean intakeWristMotorInstalled = true;
    public static boolean intakeRollerInstalled = true;

    // TOF Sensor Hardware
    // Important! You must disable any TOF sensor that is not installed!!
    // If you try to configure a TOF sensor that is not installed
    // the other TOF sensors that are installed will not work.

    // LED Hardware
    public static boolean LEDSInstalled = false;
}
