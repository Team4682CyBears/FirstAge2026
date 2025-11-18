package frc.theRobot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.theRobot.common.TofSensor;

public class ImplementationTOF extends SubsystemBase{
    private TofSensor reefCoralSensor;
    private TofSensor reefCoralSensor2;

    public ImplementationTOF(int CANId, int CANId2){
        this.reefCoralSensor = new TofSensor(CANId);
        this.reefCoralSensor2 = new TofSensor(CANId2);
        System.out.print("Implemenetation tof attempted to configure!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    public boolean isCoralDetected(){
        return reefCoralSensor.isCoralDetected();
    }

    public boolean isCoralDetected2(){
        return reefCoralSensor2.isCoralDetected();
    }

    @Override
    public void periodic() {
        if(this.reefCoralSensor != null){
            this.reefCoralSensor.publishTelemetery();
        }
        if(this.reefCoralSensor2 != null){
            this.reefCoralSensor2.publishTelemetery();
        }
    }

}
