// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: TofSensorCTRE.java
// ************************************************************

package frc.robot.common;
//import frc.robot.common.TofType;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TofSensor {

    private CANrange rangeSensor;
    private TimeOfFlight tofSensor;
    private LaserCan laserSensor;
    private TofType tofType;

    public TofSensor(TofType type, int canID) {
        this.tofType = type;
        //rangeSensor = new CANRange(canID)
        boolean configflag = true;

        switch(type){
            case CTRE:
                rangeSensor = new CANrange(canID);
                break;
            case PWF:
                tofSensor = new TimeOfFlight(canID);
                tofSensor.setRangingMode(RangingMode.Short, 20);
                break;
            case Laser:
                laserSensor = new LaserCan(canID);
                try {
                    laserSensor.setRangingMode(LaserCan.RangingMode.SHORT);
                    laserSensor.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
                } catch (ConfigurationFailedException e){
                    System.out.println("LaserTof Configuration FAILED! " + e);
                    configflag = false;
                }
                break;
        }
        if (configflag){
            System.out.println("Tof Configuration set!")
        } 
    }

    /**
    * Blinks a light on ToF sensor so it can be identified
     */
    public void blinkSensor(){
        if (this.tofType == TofType.Laser) {
            tofSensor.identifySensor();
        }
    }    

    /**
     * Getter fn to display tof type
     * @return
     */
    public String getTofType() {
        return tofType.getName();
    }

    /**
     * Gets the distance in inches from the sensor
     * @return double distance in inches
     */
    public double getRangeInches() {

        

        StatusSignal<Distance> statSig = rangeSensor.getDistance();
        Distance distance = statSig.getValue();
        SmartDashboard.putNumber(tofType.getName() + "_DistanceIn", distance.in(Inches));

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
    public void publishTelemtry() {
        SmartDashboard.putNumber(tofType.getName() + " " + "Distance (in) ", getRangeInches());
        SmartDashboard.putBoolean(tofType.getName() + " " + "Distance (in) ", isRangeValid());
    }


}
