// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: TofSensorCTRE.java
// ************************************************************

package frc.robot.common;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;

import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.compound.Diff_PositionVoltage_Velocity;
import com.ctre.phoenix6.hardware.core.CoreCANcoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * The TofSesorCTRE class helps to create TOF sensor subsystems using CTRE CANcoder and CANrange
 * which allow the rest of the code to access the TOF. It's used to sense how far away a object is from the TOF sensor in meters.
 */
public class TofSensorCTRE {
    
    private CANcoder tofSesor;
    private String displayName;
    private CANrange rangeSensor;
    private int rangeSensorCanId;


    /**
     * Contructor method for TofSensorCTRE
     * @param rangeSensorCanId int
     * @param MaxRange double
     */

    // make constructor take in only CANID. 
    // get max range as a class variable. 
    // for now, assume main canBus. so don't use canbus parameter.
    public TofSensorCTRE(int rangeSensorCanId) {
        // get rid pf cancoder here. 
        rangeSensor = new CANrange(rangeSensorCanId);
        this.rangeSensorCanId = rangeSensorCanId;
        this.displayName = "TofSesorCTRE_" + rangeSensorCanId;
    }

  
/**
     * Gets the distance in inches from the TOF sensor
     * @return double distance in inches
     */
    public double getRangeInches() {
        StatusSignal<Distance> statSig = rangeSensor.getDistance();
        Distance distance = statSig.getValue();
        SmartDashboard.putNumber(displayName + "_DistanceIn", distance.in(Inches));
        return (distance.in(Inches));
    }

    /**
     * Returns true if an object is in range
     * @return boolean (true if range is valid (0 to MAX_RANGE_INCHES))
     * 
     */

    public boolean isRangeValid() {
        double distance = getRangeInches();
        return (distance >= 0 && distance <= Constants.MAX_RANGE_INCHES);
    }

    /**
     * Publishes telemetry data to the SmartDashboard
     * Publishes distance in inches and range valid boolean
     */
    public void publishTelemetery(){
        SmartDashboard.putNumber(displayName + " DistanceIn ", getRangeInches());
        SmartDashboard.putBoolean(displayName + " RangeValid ", isRangeValid());
        //SmartDashboard.putString(displayName, " ToF status " + CANBus.CANBusStatus + rangeSensorCanId)
    }
    
}
