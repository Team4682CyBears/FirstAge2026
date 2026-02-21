// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: VisionMeasurement.java
// Intent: Forms util class to contain a vision measurement and a timestamp.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * forms a class to contain a vision measurment and a timestamp
 */
public class VisionMeasurement {
    private Pose2d robotPosition;
    private double timestamp;

    /**
     * constructor for a vision measurement
     *
     * @param position         - the robot position
     * @param timestampSeconds - the timestamp in miliseconds
     */
    public VisionMeasurement(Pose2d position, double timestampSeconds) {
        robotPosition = position;
        timestamp = timestampSeconds;
    }

    /**
     * method that returns the robot position.
     */
    public Pose2d getRobotPosition() {
        return robotPosition;
    }

    /**
     * method that returns the timestamp in seconds.
     */
    public double getTimestamp() {
        return timestamp;
    }

    /**
     * method that sets the robot position.
     * 
     * @param position - the robot position.
     */
    public void setRobotPosition(Pose2d position) {
        robotPosition = position;
    }

    /**
     * method that sets the timestamp in seconds.
     * 
     * @param timestampSeconds - the timestamp in seconds
     */
    public void setTimestamp(double timestampSeconds) {
        timestamp = timestampSeconds;
    }

}