package frc.robot.common;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants; 
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import com.ctre.phoenix6.StatusSignal;

import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The TofSensorLazer creates a new LazerCan object and is named ToFSensorLazer_canID
 * It has methods to get the distance in meters < MAX_RANGE_METERS, check if the range is valid, and publish telemetry data to the SmartDashboard.
 */
public class TofSensorLazer {
    private LaserCan lazerSensor;
    private int canID;
    private String displayName;
    private double maxRange;

    /**
     * Constructor method for TofSensorLazer
     * Creates a new LazerCan object with canID. 
     * Sets the ranging mode to SHORT (up to 1.3 meters) 
     * Sets the timing budget to 33ms. 
     * @param canID
     */
    public TofSensorLazer(int canID){
        this.displayName = "TofSesorLazer_" + canID;
        this.maxRange = Constants.MAX_RANGE_METERS;
        this.canID = canID;
        lazerSensor = new LaserCan(canID);
        try {
            lazerSensor.setRangingMode(LaserCan.RangingMode.SHORT);
            lazerSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        } 
    }
    /**
     * Gets the distance in meters from the LazerCan sensor
     * @return distance in meters
     */
    public double getDistanceMeters() {
        Measurement distanceMM = lazerSensor.getMeasurement();
        double dMilimeters = distanceMM.distance_mm;
        SmartDashboard.putNumber(displayName + "_DistanceMM", dMilimeters);

        return (dMilimeters/100.0);
    }

    /**
     * Gets the distnace in meters and checks if its valid 
     * @return true if distance is valid and within the max range (MAX_RANGE_METERS)
     */
    public boolean isRangeValid() {
        double distance = getDistanceMeters();
        return (distance >= 0 && distance <= maxRange);
    }

    /**
     * Publishes telemetry data to the SmartDashboard.
     */
    public void publishTelemetery(){
        SmartDashboard.putNumber(displayName, getDistanceMeters());
        SmartDashboard.putBoolean(displayName, isRangeValid());
        //SmartDashboard.putString(displayName, " ToF status " + CANBus.CANBusStatus + rangeSensorCanId)
    }
}