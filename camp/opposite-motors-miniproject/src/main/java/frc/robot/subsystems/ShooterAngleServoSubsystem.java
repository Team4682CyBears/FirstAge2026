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
    private final ServoChannel channel0;
    private final ServoChannel channel1;

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

        servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 5000);

        channel0.setPowered(true);
        channel1.setPowered(true);

        channel0.setEnabled(true);
        channel1.setEnabled(true);

        channel0.setPulseWidth(1500);
        channel1.setPulseWidth(1500);
    }

    public void setPosition(ShooterAngle position){
        switch(position){
            case LEFT:
                channel0.setPulseWidth((int) Constants.servoLeftPosition);
                channel1.setPulseWidth((int) Constants.servoLeftPosition);
            case RIGHT:
                channel0.setPulseWidth((int) Constants.servoRightPosition);
                channel1.setPulseWidth((int) Constants.servoRightPosition);
            default:
                channel0.setPulseWidth((int) Constants.servoDefaultPosition);
                channel1.setPulseWidth((int) Constants.servoDefaultPosition);
        }
    };
    
    public void stop(){
        channel0.setPowered(false);
        channel1.setPowered(false);
        channel0.setEnabled(false);
        channel1.setEnabled(false);
    };
    
    public int getID(){
        return canID;
    };

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Real Motor RPM (ID: %d)".formatted(canID), left);
    }


}