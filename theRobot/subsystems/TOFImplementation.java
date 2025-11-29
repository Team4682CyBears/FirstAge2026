package frc.theRobot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.theRobot.common.TofSensor;
import frc.theRobot.Constants;

public class ImplementationTOF extends SubsystemBase {
    private TofSensor reefCoralSensor = null;
    private Spinner spinner  = null;
    private double speed = 0;

    public ImplementationTOF(){
        tofSensor = new TofSensor(Constants.TofSensorCanID);
        spinner = new Spinner();
    }

    @Override
    public void periodic() {
        if(this.tofSensor != null){
            this.tofSensor.publishTelemetery();
        }
        if(this.tofSensor != null && this.spinner != null){
            speed = (16.0 - tofSensor.getRangeInches) * 4;
            spinner.spin(speed);
        }
    }
}