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

    //////// SPINDEXER CONSTANTS ///////////
    // 10 balls per second / 4 balls per rotation
    public static final double spindexerSpeedRotationsPerMinute  = 300;
    public static final double kickerBallDetectionRangeInches = 4.0; 
    public static final int spindexerSensorLaserCanID = 27;
    public static final int spindexerTalonFXCanID = 18;
    public static final double spindexerGearRatio = 25.0; //TODO get the actual number

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

    // Total field dimensions in meters
    public static final double FIELD_LENGTH_X = 16.540988; // 651.22 inches
    public static final double FIELD_WIDTH_Y = 8.069326;   // 317.69 inches

    // Blue Hub is 182.11 inches from the alliance wall
    public static final Translation2d blueHubPosition = new Translation2d(4.625594, FIELD_WIDTH_Y / 2.0);

    // Red Hub is mirrored (Field length - Blue X)
    public static final Translation2d redHubPosition = new Translation2d(FIELD_LENGTH_X - blueHubPosition.getX(), FIELD_WIDTH_Y / 2.0);

    // meters
    public static final double shuttleOffsetFromWall = 1.5;

    // left and right are relative to (0,0) facing other end of the field
    public static final Translation2d blueLeftShuttlePosition = new Translation2d(shuttleOffsetFromWall, shuttleOffsetFromWall);
    public static final Translation2d blueRightShuttlePosition = new Translation2d(shuttleOffsetFromWall, FIELD_WIDTH_Y - shuttleOffsetFromWall);
    public static final Translation2d redLeftShuttlePosition = new Translation2d(FIELD_LENGTH_X - shuttleOffsetFromWall, shuttleOffsetFromWall);
    public static final Translation2d redRightShuttlePosition = new Translation2d(FIELD_LENGTH_X - shuttleOffsetFromWall, FIELD_WIDTH_Y - shuttleOffsetFromWall);

    // *****************************************************************
    // Physical Shooter Offsets
    public static final double shooterXOffsetFromCenterOfRobot = -.2159; // in meters, positive is forward
    public static final double shooterYOffsetFromCenterOfRobot = -.1397; // in meters, positive is to the left
    public static final Translation2d shooterOffsetFromCenterOfRobot = new Translation2d(
            shooterXOffsetFromCenterOfRobot, shooterYOffsetFromCenterOfRobot);
    public static final Rotation2d shooterYawOffset = Rotation2d.fromDegrees(-60.0); 
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
        public static final double TAG_AMBIGUITY_THRESHOLD = 0.2;

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
    // Misc Constants
    public static final double DEFAULT_PROJECTILE_TIME_OF_FLIGHT_SECONDS = 1.25;

    // ********************************************************************
    // CAN IDs
    // TODO define CAN IDs here for all non drive train components

    // ********************************************************************
    // Shooter Constants
    public static final int shooterLeadMotorCanId = 21;
    public static final int shooterFollowMotorCanId = 20;

    // Shooter RPM bounds
    public static final double SHOOTER_MIN_RPM = 0.0;
    public static final double SHOOTER_MAX_RPM = 6500.0;
    public static final double SHOOTER_PONDER_RPM = 1000.0;
    public static final double SHOOTER_CLOSE_RPM = 3000.0;
    public static final double SHOOTER_RPM_OFFSET = 200;

    // ********************************************************************
    // Kicker Constants
    public static final int kickerTalonCanId = 19;

    public static final double kickerMotorGearRatio = 3.0; // 3:1
    // Kicker RPM bounds
    public static final double KICKER_MIN_RPM = 0.0;
    public static final double KICKER_MAX_RPM = 2000.0;
    public static final double KICKER_RPM = 2000.0;

    /// ******************************
    /// Hood Constants
    public static final int hoodMotorCanID = 30;
    public static final int hoodEncoderCanID = 31;

    public static final double hoodEncoderAbsoluteOffset = 0.7220;
    public static final double hoodExtendoTolerance = 0.01;
    public static final double hoodMinPositionRotations = 0.0;
    public static final double hoodMaxPositionRotations = 0.635; 

    public static final double HOOD_CLOSE_EXTENDO_POSITION = 0.12;

    /// Intake Constants
    public static final int intakeWristMotorCanID = 17;
    public static final int intakeWristEncoderCanID = 32;

    public static final double intakeWristEncoderAbsoluteOffset = -0.17789;

    public static final double intakeWristTolerance = 0.05;
    public static final double intakeWristStartingPositionRotations = 0.586;
    public static final double intakeWristDefensivePositionRotations = 0.511;
    public static final double intakeWristAgitateStowPositionRotations = 0.3439;
    public static final double intakeWristDeployedPositionRotations = 0.0; 

    public static final int intakeRollerCanId = 16;
}