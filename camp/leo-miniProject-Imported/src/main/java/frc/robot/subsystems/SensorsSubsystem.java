package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.TofSensor;
import frc.robot.common.TofType;

public class SensorsSubsystem extends SubsystemBase {

    TofSensor CTRETofSensor;
    TofSensor PWFTofSensor;
    TofSensor LaserTofSensor;
    
    public SensorsSubsystem(boolean CTREEnabled, boolean PWFEnabled, boolean LaserEnabled) {

        //TofSensor CTRETofSensor = ((CTREEnabled) ? new TofSensor("CTRE", Constants.rangeSensorCTRECanID) : null);
        //TofSensor PWFTofSensor = ((PWFEnabled) ? new TofSensor("PWF", Constants.rangeSensorCTRECanID) : null);
        //TofSensor LaserTofSensor = ((LaserEnabled) ? new TofSensor("Laser", Constants.rangeSensorCTRECanID) : null);

        if (CTREEnabled) {
            this.CTRETofSensor = new TofSensor(TofType.CTRE, Constants.rangeSensorCTRECanID);
            this.CTRETofSensor.publishTelemtry();
        }
        if (PWFEnabled) {
            this.PWFTofSensor = new TofSensor(TofType.PWF, Constants.rangeSensorCTRECanID);
            this.PWFTofSensor.publishTelemtry();
        }
        if (LaserEnabled) {
            this.LaserTofSensor = new TofSensor(TofType.Laser, Constants.rangeSensorCTRECanID);
            this.LaserTofSensor.publishTelemtry();
        }
    }

    @Override
    public void periodic() {
        //CTRETofSensor.publishTelemtry();
        //PWFTofSensor.publishTelemtry();
        //LaserTofSensor.publishTelemtry();
    }
    
}
