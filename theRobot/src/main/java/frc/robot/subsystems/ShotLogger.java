package frc.robot.subsystems;

import edu.wpi.first.util.datalog.*;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.control.SubsystemCollection;

public class ShotLogger {
    private final SubsystemCollection subsystems;

    private final DoubleLogEntry logX, logY, logYaw, logShooterRpm, logTimestamp;
    private final IntegerLogEntry logHood, logExtendo;
    private final BooleanLogEntry logMade;

    public ShotLogger(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        DataLog log = DataLogManager.getLog();

        logTimestamp = new DoubleLogEntry(log, "/shots/timestamp");
        logX         = new DoubleLogEntry(log, "/shots/robotX");
        logY         = new DoubleLogEntry(log, "/shots/robotY");
        logYaw       = new DoubleLogEntry(log, "/shots/robotYaw");
        logShooterRpm= new DoubleLogEntry(log, "/shots/shooterRpm");
        logHood      = new IntegerLogEntry(log, "/shots/hoodPulse");
        logExtendo   = new IntegerLogEntry(log, "/shots/extendoPulse");
        logMade      = new BooleanLogEntry(log, "/shots/made");
    }

    public synchronized void logShot(boolean made) {
        double timestamp = Timer.getFPGATimestamp();
        double robotX = 0.0, robotY = 0.0, robotYaw = 0.0, shooterRPM = 0.0;
        int hoodPulse = 0, extendoPulse = 0;

        if (subsystems.isDriveTrainSubsystemAvailable()) {
            Pose2d p = subsystems.getDriveTrainSubsystem().getRobotPosition();
            if (p != null) {
                robotX = p.getX();
                robotY = p.getY();
                robotYaw = p.getRotation().getDegrees();
            }
        }
        if (subsystems.isShooterSubsystemAvailable()) {
            shooterRPM = subsystems.getShooterSubsystem().getRPM();
        }
        if (subsystems.isHoodSubsystemAvailable()) {
            hoodPulse = subsystems.getHoodSubsystem().getAnglePosition();
            extendoPulse = subsystems.getHoodSubsystem().getExtendoPosition();
        }

        logTimestamp.append(timestamp);
        logX.append(robotX);
        logY.append(robotY);
        logYaw.append(robotYaw);
        logShooterRpm.append(shooterRPM);
        logHood.append(hoodPulse);
        logExtendo.append(extendoPulse);
        logMade.append(made);
        
        DataLogManager.log("Shot logged.");
    }
}