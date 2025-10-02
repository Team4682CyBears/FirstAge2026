package frc.robot.common;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Forms a class for the TofSubsystem that detects when an object is present
 * within the detection threshold.
 */
public class ToFSensor {

    private double maximumDetectionThresholdInches;
    private double minimumDetectionThresholdInches;
    private TimeOfFlight tofSensor;
    private int canId;
    private String displayName;

    /**
     * Constructor for ToFSensor class.
     * Initializes a Time of Flight sensor with specified CAN ID and detection
     * threshold.
     * Sets the sensor to short ranging mode with 20ms sample time to match robot
     * update rate.
     * 
     * @param canId                    The CAN ID of the Time of Flight sensor
     * @param detectionThresholdInches The threshold distance (in inches) for object
     *                                 detection
     */
    public ToFSensor(int canId, double maximumDetectionThresholdInches, double minimumDetectionThresholdInches) {
        tofSensor = new TimeOfFlight(canId);
        this.maximumDetectionThresholdInches = maximumDetectionThresholdInches;
        this.minimumDetectionThresholdInches = minimumDetectionThresholdInches;
        this.canId = canId;
        this.displayName = "TOF ID " + this.canId;
        // short mode is accurate to 1.3m
        // 20ms sample time matches robot update rate
        tofSensor.setRangingMode(RangingMode.Short, 20);
        DataLogManager.log("==== DONE CONFIG of TOF SENSOR at CanID " + canId);
    }

    /**
     * A method to flash the sensor
     */
    public void blinkSensor() {
        tofSensor.identifySensor();
    }

    /**
     * A method to return the display name
     * 
     * @return - the display name
     */
    public String getDisplayName() {
        return displayName;
    }

    /**
     * A method to get the sensor range in inches
     * 
     * @return - the current range in inches
     */
    public double getRangeInches() {
        return Units.metersToInches(tofSensor.getRange() / 1000);
    }

    /**
     * A method to return the standard deviation of the measurement
     * 
     * @return standard deviation in millimeters
     */
    public final double getRangeSigma() {
        return tofSensor.getRangeSigma();
    }

    /**
     * A method to detect the presence of an object
     * 
     * @return true if an object is detected
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
     * A method to return the sensor status
     * 
     * @return true if the sensor correctly measured the distance
     */
    public boolean isRangeValid() {
        return tofSensor.isRangeValid();
    }
}