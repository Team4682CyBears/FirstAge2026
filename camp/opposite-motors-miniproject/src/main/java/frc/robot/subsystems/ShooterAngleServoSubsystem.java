// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ShooterAngleServoSubsystem.java
// Intent: Changes of the shooter
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// Quick Citation Links
// https://docs.revrobotics.com/revlib/servo-hub/configuring-a-servo-hub

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import frc.robot.common.ShooterAngle;
import frc.robot.Constants;

public class ShooterAngleServoSubsystem extends ServoSubsystem{

    private final int canID;
    private final ServoHubConfig config;
    private final ServoHub servoHub;
    private ServoChannel channel0;
    private ServoChannel channel1;
    private int setPosition = 500;
    private boolean isEnabled = true;

    public ShooterAngleServoSubsystem(int canID){
        this.canID = canID; 
        this.config = new ServoHubConfig();
        this.servoHub = new ServoHub(this.canID);
        this.channel0 = servoHub.getServoChannel(ChannelId.kChannelId0);
        this.channel1 = servoHub.getServoChannel(ChannelId.kChannelId1);

        configureServos();
        servoHub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
    }

    private void configureServos() {
        config.channel0.pulseRange(1000, 1500, 2000);
        config.channel1.pulseRange(1000, 1500, 2000);
        
        channel0.setPowered(isEnabled);
        channel1.setPowered(isEnabled);

        channel0.setEnabled(isEnabled);
        channel1.setEnabled(isEnabled);

        channel0.setPulseWidth((int) Constants.servoDefaultPosition);
        channel1.setPulseWidth((int) Constants.servoDefaultPosition);

        servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 20000);

        servoHub.configure(config, ServoHub.ResetMode.kResetSafeParameters);
    }

    public void setPosition(ShooterAngle position){
        //configureServos();

        //System.out.println("SET POSITION " + position);

        switch(position){
            case LEFT:
                System.out.println("left position");
                // Actuator Period Defined here: https://www.actuonix.com/assets/images/datasheets/ActuonixL16datasheet.pdf?
                //servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 1000);
                setPosition = (int) Constants.servoLeftPosition;
                break;
            case RIGHT:
                System.out.println("right position");
                //servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 2000);
                setPosition = (int) Constants.servoRightPosition;
                break;
            case CUSTOM:
                System.out.println("custom position");
                //servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 2000);
                setPosition = (int) Constants.servoCustomPosition;
                break;
            default:
                setPosition = (int) Constants.servoDefaultPosition;
                break;
        }

        channel0.setPulseWidth(setPosition);
        System.out.println(setPosition);
    };

    public void setValue(int position){
        setPosition = MathUtil.clamp(position, 1000, 2000);
    }
    
    public void stop(){
        //isEnabled = false;
        if (!isEnabled) {
            channel0.setPowered(isEnabled);
            channel1.setPowered(isEnabled);
            channel0.setEnabled(isEnabled);
            channel1.setEnabled(isEnabled);
        }
    };
    
    public int getID(){
        return canID;
    };

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Real Motor RPM (ID: %d)".formatted(canID), left);
        if (isEnabled) {
            //System.out.println("Run to position in periodic " + setPosition);
            channel0.setPulseWidth(setPosition);
            channel1.setPulseWidth(setPosition);
            //System.out.println("Set Position CH0 " + channel0.getPulseWidth());
            //System.out.println("Set Position CH1 " + channel1.getPulseWidth());
        }

    }
}