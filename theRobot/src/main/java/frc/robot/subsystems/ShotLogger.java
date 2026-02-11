// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShotLogger.java
// Intent: Logs information about each shot
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.SubsystemCollection;

/**
 * ShotLogger is a lightweight helper that records information about each shot to the
 * WPILib DataLog. Each shot entry contains a timestamp, robot pose (x, y, yaw),
 * shooter RPM, hood and extendo encoder/pulse positions, and whether the shot was
 * made.
 *
 * <p>Usage:
 * - Create one ShotLogger instance (typically in RobotContainer or a subsystem helper)
 * - Call {@link #logShot(boolean)} when a shot event occurs (pass true if made)
 *
 * <p>Thread-safety: {@link #logShot(boolean)} is synchronized to allow safe calls from
 * different threads (for example, a command and an interrupting event).
 *
 * <p>Data paths written to the DataLog:
 * - /shots/timestamp (double)
 * - /shots/robotX (double)
 * - /shots/robotY (double)
 * - /shots/robotYaw (double)
 * - /shots/shooterRpm (double)
 * - /shots/hoodPulse (int)
 * - /shots/extendoPulse (int)
 * - /shots/made (boolean)
 */
public class ShotLogger {
    /** Reference to the project's subsystem collection for safely querying subsystem state. */
    private final SubsystemCollection subsystems;

    // DataLog entries for structured shot logging. Names chosen to be short and stable.
    private final DoubleLogEntry logX, logY, logYaw, logShooterRpm, logTimestamp;
    private final IntegerLogEntry logHood, logExtendo;
    private final BooleanLogEntry logMade;

    /**
     * Construct a ShotLogger backed by the provided SubsystemCollection.
     *
     * @param subsystems a SubsystemCollection used to read drivetrain, shooter and hood state
     */
    public ShotLogger(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        DataLog log = DataLogManager.getLog();

        // Create entries under the /shots prefix. Appending values will write a timestamped
        // entry into the robot log (suitable for later analysis or replay).
        logTimestamp = new DoubleLogEntry(log, "/shots/timestamp");
        logX         = new DoubleLogEntry(log, "/shots/robotX");
        logY         = new DoubleLogEntry(log, "/shots/robotY");
        logYaw       = new DoubleLogEntry(log, "/shots/robotYaw");
        logShooterRpm= new DoubleLogEntry(log, "/shots/shooterRpm");
        logHood      = new IntegerLogEntry(log, "/shots/hoodPulse");
        logExtendo   = new IntegerLogEntry(log, "/shots/extendoPulse");
        logMade      = new BooleanLogEntry(log, "/shots/made");
    }

    /**
     * Log a shot event to the DataLog.
     *
     * <p>This method will attempt to read the current robot pose from the drivetrain,
     * the current shooter RPM, and hood/extendo positions. If a subsystem is not
     * available or returns null, the corresponding values will be recorded as 0.
     *
     * @param made true if the shot was successful (scored), false otherwise
     */
    public synchronized void logShot(boolean made) {
        double timestamp = Timer.getFPGATimestamp();
        double robotX = 0.0, robotY = 0.0, robotYaw = 0.0, shooterRPM = 0.0;
        int hoodPulse = 0, extendoPulse = 0;

        // Safely query drivetrain for robot pose. Defensive null checking is used
        // because some startup sequences may not have valid pose information yet.
        if (subsystems.isDriveTrainSubsystemAvailable()) {
            Pose2d p = subsystems.getDriveTrainSubsystem().getRobotPosition();
            if (p != null) {
                robotX = p.getX();
                robotY = p.getY();
                robotYaw = p.getRotation().getDegrees();
            }
        }

        // Shooter RPM (may be 0 if shooter subsystem is absent)
        if (subsystems.isShooterSubsystemAvailable()) {
            shooterRPM = subsystems.getShooterSubsystem().getRPM();
        }

        // Hood & extendo encoder/pulse positions
        if (subsystems.isHoodSubsystemAvailable()) {
            hoodPulse = subsystems.getHoodSubsystem().getAnglePosition();
            extendoPulse = subsystems.getHoodSubsystem().getExtendoPosition();
        }

        // Append values to the log in a stable order. Consumers of the log can read
        // the same paths to reconstruct shot events.
        logTimestamp.append(timestamp);
        logX.append(robotX);
        logY.append(robotY);
        logYaw.append(robotYaw);
        logShooterRpm.append(shooterRPM);
        logHood.append(hoodPulse);
        logExtendo.append(extendoPulse);
        logMade.append(made);
    }
}