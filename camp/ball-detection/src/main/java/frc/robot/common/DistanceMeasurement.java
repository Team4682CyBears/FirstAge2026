// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: DistanceMeasurement.java
// Intent: Forms util class to contain a bool of if you see the correct tag and the distance from it
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

/**
 * forms a class to contain a bool of tag is valid and distance in meters from
 * tag
 */
public class DistanceMeasurement {
    private boolean tagIsValid;
    private double distanceMeters;

    /**
     * constructor for a vision measurement
     *
     * @param tagIsValid     - is the correct tag seen
     * @param distanceMeters - the distance in meters
     */
    public DistanceMeasurement(boolean tagIsValid, double distanceMeters) {
        this.tagIsValid = tagIsValid;
        this.distanceMeters = distanceMeters;
    }

    /**
     * method that returns the valid.
     */
    public boolean getIsValid() {
        return tagIsValid;
    }

    /**
     * method that returns the distance in meteres.
     */
    public double getDistanceMeters() {
        return distanceMeters;
    }

    /**
     * method that sets the robot position.
     * 
     * @param tagIsValid - the robot position.
     */
    public void setIsValid(boolean tagIsValid) {
        this.tagIsValid = tagIsValid;
    }

    /**
     * method that sets the timestamp in seconds.
     * 
     * @param distanceMeters - the timestamp in seconds
     */
    public void setDistanceMeteres(double distanceMeters) {
        this.distanceMeters = distanceMeters;
    }

}