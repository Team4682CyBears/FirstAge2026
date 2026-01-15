package frc.robot.common;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants; 
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;

import com.ctre.phoenix6.StatusSignal;

import au.grapplerobotics.ConfigurationFailedException;
import edu.wpi.first.wpilibj.TimedRobot;


public class TofSensorLazer {
    private LaserCan lazerSensor;
    private int canID;
    private String displayName;
    private double maxRange;

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

    public double getDistanceMeters() {
        Measurement distanceMM = lazerSensor.getMeasurement();
        double dMilimeters = distanceMM.distance_mm;
        SmartDashboard.putNumber(displayName + "_DistanceMM", dMilimeters);

        return (dMilimeters/100.0);
    }

    public boolean isRangeValid() {
        double distance = getDistanceMeters();
        return (distance >= 0 && distance <= maxRange);
    }


    public void publishTelemetery(){
        SmartDashboard.putNumber(displayName, getDistanceMeters());
        SmartDashboard.putBoolean(displayName, isRangeValid());
        //SmartDashboard.putString(displayName, " ToF status " + CANBus.CANBusStatus + rangeSensorCanId)
    }
}