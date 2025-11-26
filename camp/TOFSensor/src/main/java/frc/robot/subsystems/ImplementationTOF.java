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

    /**
     * The constructor for ImplementationTOF it has no parameters 
    */
    public ImplementationTOF(){
        tofSensor = new TofSensor(Constants.TofSensorCanID);
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
            speed = (16.0 - tofSensor.getRangeInches());
            if(speed < 0){
                speed = 0;
            }
            spinner.spin(speed);
        }else{
            spinner.spin(0);
        }
    }
}