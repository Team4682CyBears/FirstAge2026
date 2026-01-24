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
import frc.robot.common.TofSesorCTRE;

/**
 * The Implementation class is used to access the tof and motors and then
 * uses a periodic function in order to actually implement the functionality 
 * of the project
 */

public class Implementation extends SubsystemBase{
    private TofSesorCTRE tofSensorCTRE;
    private TofSensorPWF tofSensorPWF;
    private TofSensorLaser tofSensorLaser;
    private Spinner spinner;
    private double speed = 0;

    /**
     * Constructor for Implementation class that takes 
     * @param useCTRE boolean to determine whether to create CTRE TOF sensor
     * @param usePWF boolean to determine whether to create PWF TOF sensor
     * @param useLaser boolean to determine whether to create Laser TOF sensor
     */
    public Implementation(boolean useCTRE, boolean usePWF, boolean useLaser){
        if (useCTRE){
            tofSensorCTRE = new TofSesorCTRE(10);
        } 
        if (usePWF){
            tofSensorPWF = new TofSensorPWF(5);
        }
        if (useLaser){
            tofSensorLaser = new TofSensorLaser(15);
        }
        spinner = new Spinner(Constants.SPINNER_CAN_ID);
    }

    public void setMotorSpeed(double speed){
        this.speed = speed;
        //spinner.spin(speed);
    }
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
            setMotorSpeed(10);
        }
    }

    
}
