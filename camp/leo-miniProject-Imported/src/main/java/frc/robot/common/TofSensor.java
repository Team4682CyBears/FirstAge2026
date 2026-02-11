package frc.robot.common;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TofSensor {

    private CANrange rangeSensor;
    private String displayName;

    public TofSensor(String displayName, int canID) {
        this.displayName = displayName;
        //rangeSensor = new CANRange(canID)
    }

    /**
     * Gets the distance in inches from the sensor
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
    public void publishTelemtry() {
        SmartDashboard.putNumber(displayName + " " + "Distance (in) ", getRangeInches());
        SmartDashboard.putBoolean(displayName + " " + "Distance (in) ", isRangeValid());
    }


}
