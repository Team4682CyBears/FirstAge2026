package frc.robot.common;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * The TofSesorCTRE class helps to create TOF sensor subsystems using CTRE CANcoder and CANrange
 * which allow the rest of the code to access the TOF. It's used to sense how far away a object is from the TOF sensor in meters.
 */
public class TofSesorCTRE {
    
    private CANcoder tofSesor;
    private CANBus canBus;
    private int tofSensorCanId;
    private String displayName;
    private CANrange rangeSensor;
    private int rangeSensorCanId;


    /**
     * Contructor method for TofSensorCTRE
     * @param tofSensorCan
     * @param rangeSensorCanId
     * @param canBus
     * @param MaxRange
     */
    public TofSesorCTRE(int tofSensorCan, int rangeSensorCanId, CANBus canBus, double MaxRange) {
        tofSesor = new CANcoder(tofSensorCanId, canBus);
        rangeSensor = new CANrange(rangeSensorCanId, canBus);
        this.canBus = canBus;
        this.tofSensorCanId = tofSensorCanId;
        this.rangeSensorCanId = rangeSensorCanId;
        this.displayName = "TofSesor_" + tofSensorCanId;
    }

    /**
     * Gets the distance in metesr from the CANrange sensor
     * @return distance in meters
     */
    public double getDistanceMeters() {
        StatusSignal statSig = rangeSensor.getDistance();
        double distanceMM = statSig.getValueAsDouble();
        SmartDashboard.putNumber(displayName + "_DistanceMM", distanceMM);
        return (distanceMM/100.0);
    }
    
    

    public boolean isRangeValid() {
        double distance = getDistanceMeters();
        return (distance >= 0 && distance <= Constants.MAX_RANGE_METERS);
    }


    public void publishTelemetery(){
        SmartDashboard.putNumber(displayName, getDistanceMeters());
        SmartDashboard.putBoolean(displayName, isRangeValid());
        //SmartDashboard.putString(displayName, " ToF status " + CANBus.CANBusStatus + rangeSensorCanId)
    }

    
    
}
