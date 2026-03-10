// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: TofSensorLaser.java
// ************************************************************

package frc.robot.common;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.units.measure.Distance;
import au.grapplerobotics.LaserCan;

import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import au.grapplerobotics.ConfigurationFailedException;

/**
 * The TofSensorlaser creates a new laserCan object and is named ToFSensorlaser_canID
 * It has methods to get the distance in meters < MAX_RANGE_METERS, check if the range is valid, and publish telemetry data to the SmartDashboard.
 */
public class TofSensorLaser {
    private LaserCan laserSensor;
    private String displayName;
    private double maxRange = Meters.of(1.3).in(Inches);
    private double detectionDistanceInches;

    /**
     * Constructor method for TofSensorlaser
     * Creates a new laserCan object with canID. 
     * Sets the ranging mode to SHORT (up to 1.3 meters) 
     * Sets the timing budget to 33ms. 
     * @param canID
     */
    public TofSensorLaser(int canID, double detectionDistanceInches){
        this.detectionDistanceInches = detectionDistanceInches;
        this.displayName = "TofSesorLaser_" + canID;
        laserSensor = new LaserCan(canID);
        try {
            laserSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            laserSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        } 
        System.out.println("TOF CONFIGURED"); 
    }

    /**
     * Gets the distance in meters from the laserCan sensor
     * @return distance (double) in inches
     * 
     */ 
    // use wpilib units to convert to inches. metersToInches and inchesToMeters
    public double getRangeInches() {
        Measurement measurement = laserSensor.getMeasurement();
        return getRangeInches(measurement);
    }

    /**
     * Checks if object is detected
     * @return true if distance is valid and within the max range (MAX_RANGE_METERS)
     */
    public boolean isDetected() {
        Measurement measurement = laserSensor.getMeasurement();
        double distance = getRangeInches(measurement);
        return (isValid(measurement) && distance >= 0 && distance <= detectionDistanceInches);
    }

    /**
     * Publishes telemetry data to the SmartDashboard.
     * Publishes distance in meters and if the range is valid
     */
    public void publishTelemetery(){
        SmartDashboard.putNumber(displayName + " DistanceIn ", getRangeInches());
        SmartDashboard.putBoolean(displayName + "tofActivated ", isDetected());
    }

    private double getRangeInches(Measurement measurement){
        if (isValid(measurement)){
            Distance distance = Millimeters.of(measurement.distance_mm);
            return (distance.in(Inches));
        }
        // if invalid, return the max range as a default
        return maxRange;
    }

    private boolean isValid(Measurement measurement) {
        return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
    }
}