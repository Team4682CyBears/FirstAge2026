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
    }

    private void configureServos() {
        config.channel0.pulseRange(500, 1500, 2500);
        config.channel1.pulseRange(500, 1500, 2500);
        
        channel0.setPowered(isEnabled);
        channel1.setPowered(isEnabled);

        channel0.setEnabled(isEnabled);
        channel1.setEnabled(isEnabled);

        channel0.setPulseWidth((int) Constants.servoDefaultPosition);
        channel1.setPulseWidth((int) Constants.servoDefaultPosition);
    }

    public void setPosition(ShooterAngle position){
        configureServos();

        switch(position){
            case LEFT:
                // Actuator Period Defined here: https://www.actuonix.com/assets/images/datasheets/ActuonixL16datasheet.pdf?
                servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 1000);
                setPosition = (int) Constants.servoLeftPosition;
                break;
            case RIGHT:
                servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 2000);
                setPosition = (int) Constants.servoRightPosition;
                break;
            default:
                setPosition = (int) Constants.servoDefaultPosition;
                break;
        }
    };
    
    public void stop(){
        //isEnabled = false;
        channel0.setPowered(isEnabled);
        channel1.setPowered(isEnabled);
        channel0.setEnabled(isEnabled);
        channel1.setEnabled(isEnabled);
    };
    
    public int getID(){
        return canID;
    };

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Real Motor RPM (ID: %d)".formatted(canID), left);
        if (isEnabled) {
            channel0.setPulseWidth(setPosition);
            channel1.setPulseWidth(setPosition);
        }
    }
}