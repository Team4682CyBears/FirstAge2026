package frc.theRobot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.theRobot.common.TofSensor;
import frc.theRobot.Constants;

public class ImplementationTOF extends SubsystemBase {
    private TofSensor reefCoralSensor = null;

    public ImplementationTOF(){
        reefCoralSensor = new TofSensor(Constants.TofSensorCanID);
    }

    @Override
    public void periodic() {
        if(this.reefCoralSensor != null){
            this.reefCoralSensor.publishTelemetery();
        }
    }
}