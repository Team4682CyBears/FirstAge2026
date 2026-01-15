// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: TofSensor.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ package frc.robot.common;
package frc.robot.common;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * The TofSensor class helps to create TOF sensor subsystems which allow the
* rest of the code to access the TOF. It's used to sense objects and how far away
* the objects are from the TOF sensor.
*/
public class TofSensor {
    private TimeOfFlight tofSensor;
    private int canID;
    private String displayName;

    public TofSensor(int canID){
        tofSensor = new TimeOfFlight(canID);
        this.canID = canID;
        this.displayName = "TOF ID" + this.canID;

        tofSensor.setRangingMode(RangingMode.Short, 20);
        System.out.println("DONE CONFIG OF TOF SENSOR AT CanID!!!" + canID);
    }


/**
 * Blinks a light on the TOF sensor so it can be identified
 */
    public void blinkSensor(){
        tofSensor.identifySensor();
    }

/**
 * getter function for the display name variable
 */
    public String getDisplayName(){
        return displayName;
    }
/**
 * Gets the number of inches that a object is away from the TOF sensor
 */
    public double getRangeInches(){
        return Units.metersToInches(tofSensor.getRange()/1000);   
    }

/**
 * Returns true if an object is in range
 * 
 */
    public boolean isRangeValid(){
        return tofSensor.isRangeValid();
    }

/**
 * Publishes the telemetry to the smartdashboard
 */
    public void publishTelemetery(){
        SmartDashboard.putNumber(displayName + " Range Inches" , this.getRangeInches());
        SmartDashboard.putBoolean(displayName + " Range Is Valid", this.isRangeValid());
        SmartDashboard.putString(displayName + " TOF Status", this.tofSensor.getStatus().toString());
        //System.out.println("TOF telemtry published!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      }
    
}



