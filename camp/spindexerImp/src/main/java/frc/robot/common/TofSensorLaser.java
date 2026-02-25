// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: TofSensorLaser.java
// ************************************************************

package main.java.frc.robot.common;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants; 
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.StatusSignal;

import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;

/**
 * The TofSensorlaser creates a new laserCan object and is named ToFSensorlaser_canID
 * It has methods to get the distance in meters < MAX_RANGE_METERS, check if the range is valid, and publish telemetry data to the SmartDashboard.
 */
public class TofSensorLaser {
    private LaserCan laserSensor;
    private int canID;
    private String displayName;
    private double maxRange;

    /**
     * Constructor method for TofSensorlaser
     * Creates a new laserCan object with canID. 
     * Sets the ranging mode to SHORT (up to 1.3 meters) 
     * Sets the timing budget to 33ms. 
     * @param canID
     */
    public TofSensorLaser(int canID){
        this.displayName = "TofSesorLaser_" + canID;
        this.maxRange = Constants.MAX_RANGE_INCHES;
        this.canID = canID;
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


    /**
     * Gets the distnace in meters and checks if its valid 
     * @return true if distance is valid and within the max range (MAX_RANGE_METERS)
     * Also checks if its ac
     */
    public boolean tofActivated() {
        Measurement measurement = laserSensor.getMeasurement();
        
        //use the code:
        // laserSensor.getMeasurement();
        // returnmeasurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT
        double distance = getRangeInches();
        return (distance >= 0 && measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && distance <= maxRange);
    }

    /**
     * Publishes telemetry data to the SmartDashboard.
     * Publishes distance in meters and if the range is valid
     */
    public void publishTelemetery(){
        SmartDashboard.putBoolean(displayName + "tofActivated ", tofActivated());
        //martDashboard.putString(displayName, " ToF status " + CANBus.CANBusStatus + rangeSensorCanId)
    }
}