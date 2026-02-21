// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;

    //////// SWERVE MODULE CONFIGS ///////////
    public static final double SWERVE_MAX_SPEED = 5.0; // m/s
    // We got 657 from path planner
    public static final double SWERVE_MAX_ANGULAR_SPEED = Rotation2d.fromDegrees(657).getRadians(); // rad/s

    // *****************************************************************
    // Auto Constants
    public static final PPHolonomicDriveController pathFollower = new PPHolonomicDriveController(
            new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(4.5, 0.001, 0.0) // Rotation PID constants
    );

    public static final PathConstraints autoAlignPathConstraints = new PathConstraints(3.0, 2.0, 540, 720);
    // Diagnostic Constants
    public static final boolean putDiagnosticPaths = true;

    // *****************************************************************
    // Field Constants (These are in field-position, not FMAP coordinates)

    // This is calculated using the bottom left corner of the field as (0,0). The y
    // coordinate is the height of the field divided by 2 and the x is the distance
    // to the center of the hub and the width of the field minus the distance to the
    // center of the hub. These are all in meters
    public static final Translation2d blueHubPosition = new Translation2d(4.625594, 8.069326 / 2); // removed adding one
                                                                                                   // to this
    public static final Translation2d redHubPosition = new Translation2d(16.540988 - 4.625594, 8.069326 / 2);

    // *****************************************************************
    // Physical Shooter Offsets
    public static final double shooterXOffsetFromCenterOfRobot = 0.0; // in meters, positive is forward
    public static final double shooterYOffsetFromCenterOfRobot = 0.0; // in meters, positive is to the left
    public static final double shooterYawOffset = 0.0; // in degrees
    // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // NEO 550 maximum RPM - see:
    // https://www.revrobotics.com/rev-21-1651/#:~:text=The%20following%20specifications%20for%20the%20NEO%20550%20Brushless,Motor%20Kv%3A%20917%20Kv%20Free%20Speed%3A%2011000%20RPM
    public static final double neoFiveFiveZeroMaximumRevolutionsPerMinute = 11000;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per
    // rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42;
    // CTRE motor constants
    public static final double talonMaximumRevolutionsPerMinute = 6380;

    // Motor Constants for End Effector and Elevator
    public static final double motorStatorCurrentMaximumAmps = 100.0;
    public static final double motorSupplyCurrentMaximumAmps = 50.0;

    public static final double motorSupplyVoltageTimeConstant = 0.02;
    public static final double falconMaxVoltage = 12.0;

    // *****************************************************************
    // input device constants
    public static final int portDriverController = 0;
    public static final int portCoDriverController = 1;
    public static final int climberLimSwtichChannel = 1;

    // ******************************************************************
    // led constants
    public static final int ledPWMID = 0;
    public static final int ledLength = 30;
    public static final int ledStartIdx = 0;
    public static final int ledBlinkFrquencyInHertz = 2;
    public static final double ledBrightness = 0.5;

    // ******************************************************************
    // camera constants
    public static final boolean useFusedVisionInAuto = false;
    public static final double autoUseFusedVisionDuration = 15.0;

    public static final double limelightToWPIBlueXOffest = 8.75;
    public static final double limelightToWPIBlueYOffset = 4.0;

    // Threshold for limelight/AprilTag pose ambiguity above which detections are
    // considered ambiguous.
    public static final double TAG_AMBIGUITY_THRESHOLD = 0.6;

    public static final double IMUassistAlpha = .01; // value between 0 and 1, higher values will cause the IMU to have
                                                     // more influence on the final angle output

    // ********************************************************************
    // Controller Constants
    public static final double rumbleTimeSeconds = 0.15;

    // ********************************************************************
    // PowerDistributionPanel Constants
    public static final int currentPowerDistributionPanelCanId = 29;
    public static final ModuleType currentPowerDistributionPanelType = ModuleType.kRev;
    public static final double overcurrentRumbleTimeSeconds = 0.25;

    // ********************************************************************
    // CAN IDs
    // TODO define CAN IDs here for all non drive train components

    // shooter related can ids
    public static final int shooterLeadMotorCanId = 21;
    public static final int shooterFollowMotorCanId = 20;

    // kicker can ids and constants
    public static final int kickerTalonCanId = 19;

    public static final double kickerMotorGearRatio = 1;

    /// ******************************
    /// Hood Constants
    public static final int hoodMotorCanID = 30;
    public static final int hoodEncoderCanID = 31;

    public static final double hoodEncoderAbsoluteOffset = -0.417;
    public static final double hoodExtendoTolerance = 0.01;
    public static final double hoodMinPositionRotations = 0.0;
    public static final double hoodMaxPositionRotations = 0.635; // TODO measure this on device

    public static final int servoDefaultPosition = 1000; // fully retracted position

    // Shooter RPM bounds
    public static final double SHOOTER_MIN_RPM = 0.0;
    public static final double SHOOTER_MAX_RPM = 5000.0;
    // ********************************************************************

    public static final double PROJECTILE_TIME_OF_FLIGHT_SECONDS = 1.49;

    // Hood servo pulse widths
    public static final int HOOD_MIN_PULSE = 1000;
    public static final int HOOD_MAX_PULSE = 2000;
    // Distances corresponding to min and max hood positions
    public static final double HOOD_MIN_DISTANCE_METERS = 1.0;
    public static final double HOOD_MAX_DISTANCE_METERS = 8.0;

}