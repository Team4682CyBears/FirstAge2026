package frc.robot.subsystems;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HoodSubsystem extends SubsystemBase {
    private float targetHoodExtension = 0; // Inches

    private final DoubleLogEntry hoodExtensionLogEntry;

    public HoodSubsystem() {
        this.hoodExtensionLogEntry = new DoubleLogEntry(DataLogManager.getLog(), "/shooter/hoodExtension");
        // TODO: Implement
    }

    public void setHoodExtension(float inches) {
        this.targetHoodExtension = inches;
    }

    @Override
    public void periodic() {
        hoodExtensionLogEntry.append(0.0); // TODO: Change to real value
    }
}
