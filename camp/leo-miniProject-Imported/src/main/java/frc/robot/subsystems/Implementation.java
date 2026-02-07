// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: Implementation.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.TofSensorLaser;
import frc.robot.common.TofSensorPWF;
import frc.robot.common.TofSensorCTRE;

/**
 * The Implementation class is used to access the tof and motors and then
 * uses a periodic function in order to actually implement the functionality 
 * of the project
 */

public class Implementation extends SubsystemBase{
    private TofSensorCTRE tofSensorCTRE;
    private TofSensorPWF tofSensorPWF;
    private TofSensorLaser tofSensorLaser;
    private Spinner spinner;
    private double speed = 0;

    /**
     * Constructor for Implementation class that takes 
     * @param useCTRE boolean to determine whether to create CTRE TOF sensor
     * @param usePWF boolean to determine whether to create PWF TOF sensor
     * @param useLaser boolean to determine whether to create Laser TOF sensor
     * @param useSpinner boolean to determine whether to create Spinner motor
     */
    public Implementation(boolean useCTRE, boolean usePWF, boolean useLaser, boolean useSpinner){
        if (useCTRE){
            tofSensorCTRE = new TofSensorCTRE(Constants.rangeSensorCTRECanID);
        } 
        if (usePWF){
            tofSensorPWF = new TofSensorPWF(Constants.rangeSensorPWFCanID);
        }
        if (useLaser){
            tofSensorLaser = new TofSensorLaser(Constants.rangeSensorLaserCanID);
        }
        if (useSpinner){
            spinner = new Spinner(Constants.SPINNER_CAN_ID);
        }
    }

    public Spinner getSpinner(){
        return spinner;
    }
    /**
     * sets the speed value to the speed value that the motor will spin at (RPM)
     * @param speed double speed to set the motor to
     */
    public void setMotorSpeed(double speed){
        this.speed = speed;
        //spinner.spin(speed);
    }

    /**
     * periodic function that runs every 20ms
     * publishes telemetry for each TOF sensor (if they are created/enabled)
     * spins the motor at the set speed (if spinner is created)
     */ 
    @Override
    public void periodic(){
        if(this.tofSensorCTRE != null){
            this.tofSensorCTRE.publishTelemetery();
        }
        if(this.tofSensorPWF != null){
            this.tofSensorPWF.publishTelemetery();
        }
        if(this.tofSensorLaser != null){
            this.tofSensorLaser.publishTelemetery();
        }
        if(this.spinner != null){
            setMotorSpeed(Constants.motorSpeed);
            spinner.setRPM(speed);
        }
    }

    
}
