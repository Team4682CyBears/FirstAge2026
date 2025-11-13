// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: ToFSensor.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Simple wrapper for a Time-of-Flight (ToF) sensor.
 */
public class ToFSensor {

    private int canId;
    private double maximumDetectionThresholdInches;
    private double minimumDetectionThresholdInches;
    private String displayName;
    private TimeOfFlight tofSensor;

    /**
     * Construct a ToFSensor.
     *
     * @param canId                           CAN ID for the sensor
     * @param maximumDetectionThresholdInches maximum detection distance in inches
     * @param minimumDetectionThresholdInches minimum detection distance in inches
     */
    public ToFSensor(int canId, double maximumDetectionThresholdInches, double minimumDetectionThresholdInches) {
        this.canId = canId;
        this.maximumDetectionThresholdInches = maximumDetectionThresholdInches;
        this.minimumDetectionThresholdInches = minimumDetectionThresholdInches;
        this.displayName = "TOF ID " + this.canId;
        tofSensor = new TimeOfFlight(canId);
        // short mode is accurate to 1.3m
        // 20ms sample time matches robot update rate
        tofSensor.setRangingMode(RangingMode.Short, 20);
        DataLogManager.log("==== DONE CONFIG of TOF SENSOR at CanID " + canId);
    }

    /**
     * Flash the sensor LED to identify it.
     */
    public void blinkSensor() {
        tofSensor.identifySensor();
    }

    /**
     * Get a short display name for the sensor.
     *
     * @return display name string
     */
    public String getDisplayName() {
        return displayName;
    }

    /**
     * Get the current measured range in inches.
     *
     * @return range in inches
     */
    public double getRangeInches() {
        return Units.metersToInches(tofSensor.getRange() / 1000.0);
    }

    /**
     * Get the range measurement standard deviation (mm).
     *
     * @return standard deviation in millimeters
     */
    public double getRangeSigma() {
        return tofSensor.getRangeSigma();
    }

    /**
     * Check if an object is detected within thresholds.
     *
     * @return true if detected
     */
    public boolean isDetected() {
        if (this.isRangeValid()) {
            double currentRangeInches = this.getRangeInches();
            if ((currentRangeInches < maximumDetectionThresholdInches)
                    && (currentRangeInches > minimumDetectionThresholdInches)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Check whether the current range reading is valid.
     *
     * @return true if the sensor reports a valid range
     */
    public boolean isRangeValid() {
        return tofSensor.isRangeValid();
    }
}