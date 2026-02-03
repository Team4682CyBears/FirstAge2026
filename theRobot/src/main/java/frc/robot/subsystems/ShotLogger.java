// Simple shot logger that writes CSV entries to the robot filesystem
package frc.robot.subsystems;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.control.SubsystemCollection;

/**
 * Small utility that logs shots (made/missed) to a CSV file on the robot.
 * Columns: timestamp,robot_x_m,robot_y_m,robot_yaw_deg,shooter_rpm,hood_pulse,extendo_pulse,made
 */
public class ShotLogger {
    private final SubsystemCollection subsystems;
    private final File outFile;
    private final Object writeLock = new Object();

    public ShotLogger(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        // Use a file path on the robot persistent filesystem. LV user is standard.
        this.outFile = new File("/home/lvuser/shot_log.csv");
    }

    /**
     * Append a shot line to the CSV. Safe to call from command/button handlers.
     *
     * @param made true if shot made, false if missed
     */
    public void logShot(boolean made) {
        // gather data (use fallbacks if subsystems missing)
        double timestamp = Timer.getFPGATimestamp();

        double robotX = Double.NaN;
        double robotY = Double.NaN;
        double robotYaw = Double.NaN;
        double shooterRPM = Double.NaN;
        int hoodPulse = -1;
        int extendoPulse = -1;

        try {
            if (subsystems.isDriveTrainSubsystemAvailable()) {
                Pose2d p = subsystems.getDriveTrainSubsystem().getRobotPosition();
                if (p != null) {
                    robotX = p.getX();
                    robotY = p.getY();
                    Rotation2d r = p.getRotation();
                    if (r != null) {
                        robotYaw = r.getDegrees();
                    }
                }
            }

            if (subsystems.isShooterSubsystemAvailable()) {
                shooterRPM = subsystems.getShooterSubsystem().getRPM();
            }

            if (subsystems.isHoodSubsystemAvailable()) {
                hoodPulse = subsystems.getHoodSubsystem().getAnglePosition();
                extendoPulse = subsystems.getHoodSubsystem().getExtendoPosition();
            }
        } catch (Exception e) {
            DataLogManager.log("ShotLogger: error gathering shot data: " + e.toString());
        }

        // write CSV (append). Create header if file doesn't exist.
        synchronized (writeLock) {
            boolean needsHeader = !outFile.exists();
            try (FileWriter fw = new FileWriter(outFile, true);
                    BufferedWriter bw = new BufferedWriter(fw);
                    PrintWriter pw = new PrintWriter(bw)) {
                if (needsHeader) {
                    pw.println("timestamp,robot_x_m,robot_y_m,robot_yaw_deg,shooter_rpm,hood_pulse,extendo_pulse,made");
                }
                // safe formatting of NaNs and ints
                String line = String.format(java.util.Locale.US,
                        "%.6f,%.4f,%.4f,%.4f,%.2f,%d,%d,%b",
                        timestamp,
                        Double.isNaN(robotX) ? Double.NaN : robotX,
                        Double.isNaN(robotY) ? Double.NaN : robotY,
                        Double.isNaN(robotYaw) ? Double.NaN : robotYaw,
                        Double.isNaN(shooterRPM) ? Double.NaN : shooterRPM,
                        hoodPulse,
                        extendoPulse,
                        made);
                pw.println(line);
                DataLogManager.log("ShotLogger: logged shot -> " + line);
            } catch (IOException e) {
                DataLogManager.log("ShotLogger: failed to write shot log: " + e.toString());
            }
        }
    }
}
