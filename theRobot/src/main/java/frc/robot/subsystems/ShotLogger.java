// Simple shot logger that writes CSV entries to the robot filesystem
package frc.robot.subsystems;

import java.nio.ByteBuffer;

import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.control.SubsystemCollection;

/**
 * Small utility that logs shots (made/missed) to a CSV file on the robot.
 * Columns:
 * timestamp,robot_x_m,robot_y_m,robot_yaw_deg,shooter_rpm,hood_pulse,extendo_pulse,made
 */
public class ShotLogger {
    private final SubsystemCollection subsystems;
    private final StructLogEntry<ShotRecord> shotLogEntry;

    public ShotLogger(SubsystemCollection subsystems) {
        this.subsystems = subsystems;
        DataLog log = DataLogManager.getLog();
        this.shotLogEntry = StructLogEntry.create(log, "/shots", ShotRecord.struct);
    }

    public void logShot(boolean made) {
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
                shooterRPM = SmartDashboard.getNumber("Shooter RPM", 0);
            }

            if (subsystems.isHoodSubsystemAvailable()) {
                hoodPulse = subsystems.getHoodSubsystem().getAnglePosition();
                extendoPulse = subsystems.getHoodSubsystem().getExtendoPosition();
            }
        } catch (Exception e) {
            DataLogManager.log("ShotLogger: error gathering shot data: " + e.toString());
        }

        try {
            ShotRecord rec = new ShotRecord(timestamp,
                    Double.isNaN(robotX) ? Double.NaN : robotX,
                    Double.isNaN(robotY) ? Double.NaN : robotY,
                    Double.isNaN(robotYaw) ? Double.NaN : robotYaw,
                    Double.isNaN(shooterRPM) ? Double.NaN : shooterRPM,
                    hoodPulse,
                    extendoPulse,
                    made);

            shotLogEntry.append(rec);
            DataLogManager.log("ShotLogger: logged shot struct");
        } catch (Exception e) {
            DataLogManager.log("ShotLogger: failed to append shot struct: " + e.toString());
        }
    }

    /**
     * Immutable record representing a single shot. Includes a Struct serializer
     * so it can be logged with StructLogEntry.
     */
    private static final class ShotRecord {
        public final double timestamp;
        public final double robot_x_m;
        public final double robot_y_m;
        public final double robot_yaw_deg;
        public final double shooter_rpm;
        public final int hood_pulse;
        public final int extendo_pulse;
        public final boolean made;

        public ShotRecord(double timestamp, double robot_x_m, double robot_y_m, double robot_yaw_deg,
                double shooter_rpm, int hood_pulse, int extendo_pulse, boolean made) {
            this.timestamp = timestamp;
            this.robot_x_m = robot_x_m;
            this.robot_y_m = robot_y_m;
            this.robot_yaw_deg = robot_yaw_deg;
            this.shooter_rpm = shooter_rpm;
            this.hood_pulse = hood_pulse;
            this.extendo_pulse = extendo_pulse;
            this.made = made;
        }

        // Struct implementation for serializing/deserializing ShotRecord
        public static final Struct<ShotRecord> struct = new Struct<ShotRecord>() {
            private final int size = Struct.kSizeDouble * 5 + Struct.kSizeInt32 * 2 + Struct.kSizeBool;

            @Override
            public Class<ShotRecord> getTypeClass() {
                return ShotRecord.class;
            }

            @Override
            public String getTypeName() {
                return "ShotRecord";
            }

            @Override
            public int getSize() {
                return size;
            }

            @Override
            public String getSchema() {
                // Simple schema description, optional
                return "timestamp:double,robot_x_m:double,robot_y_m:double,robot_yaw_deg:double,shooter_rpm:double,hood_pulse:int32,extendo_pulse:int32,made:bool";
            }

            @Override
            public ShotRecord unpack(ByteBuffer bb) {
                double timestamp = bb.getDouble();
                double robot_x_m = bb.getDouble();
                double robot_y_m = bb.getDouble();
                double robot_yaw_deg = bb.getDouble();
                double shooter_rpm = bb.getDouble();
                int hood = bb.getInt();
                int ext = bb.getInt();
                boolean made = bb.get() != 0;
                return new ShotRecord(timestamp, robot_x_m, robot_y_m, robot_yaw_deg, shooter_rpm, hood, ext, made);
            }

            @Override
            public void pack(ByteBuffer bb, ShotRecord value) {
                bb.putDouble(value.timestamp);
                bb.putDouble(value.robot_x_m);
                bb.putDouble(value.robot_y_m);
                bb.putDouble(value.robot_yaw_deg);
                bb.putDouble(value.shooter_rpm);
                bb.putInt(value.hood_pulse);
                bb.putInt(value.extendo_pulse);
                bb.put((byte) (value.made ? 1 : 0));
            }
        };
    }
}
