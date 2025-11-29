// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: ImplementationTOF.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.TofSensor;

/**
 * The ImplementationTOF class is used to accsess the tof and motors and then
 * uses a periodic function in order to actually implement the functionality 
 * of the project
 */
public class ImplementationTOF extends SubsystemBase {
    private TofSensor tofSensor;
    private Spinner spinner;
    private double speed = 0;
    // when the distance is 16 inches, the motor is slowest
    private double maxDistance = 16.0;
    // reduce max speed for the test board
    private double speedDeratingFactor = 0.75;

    /**
     * The constructor for ImplementationTOF it has no parameters 
    */
    public ImplementationTOF(){
        tofSensor = new TofSensor(Constants.tofSensorCanID);
        spinner = new Spinner();
    }

    /**
     * Overrides the periodic function and then outputs telemetries for 
     * the tof sensor and the motor to the smartdashboard it also sets
     * the speed of the motor to a number inversely related to the range 
     * detected by the tofSensor.
     */
    @Override
    public void periodic() {
        if(this.tofSensor != null){
            this.tofSensor.publishTelemetery();
        }
        if(this.spinner != null){
            this.spinner.publishTelemetery();
        }
        if(this.tofSensor != null && this.spinner != null && tofSensor.isRangeValid()){
            /**
             * This math finds uses the max range of the tof sensor and how close an object 
             * is from the TOF sensor. It then uses those two numbers to find a inverse of
             * the sensed distance. The number is then divided by the max distance as the 
             * spark max only takes in numbers from 0.0 - 1.0.
             */
            speed = (maxDistance - tofSensor.getRangeInches())/maxDistance;
            if(speed < 0){
                speed = 0;
            }
            spinner.spin(speed * speedDeratingFactor);
        }else{
            spinner.spin(0);
        }
    }
}