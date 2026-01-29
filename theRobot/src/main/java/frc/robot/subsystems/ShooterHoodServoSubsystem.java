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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import frc.robot.control.Constants;

public class ShooterHoodServoSubsystem extends SubsystemBase {

    private final int canID;
    private final ServoHubConfig config;
    private final ServoHub servoHub;
    private ServoChannel channel0;
    private ServoChannel channel1;
    private int setPosition = 1500;

    public ShooterHoodServoSubsystem(int canID) {
        this.canID = canID;
        this.config = new ServoHubConfig();
        this.servoHub = new ServoHub(this.canID);
        this.channel0 = servoHub.getServoChannel(ChannelId.kChannelId0);
        this.channel1 = servoHub.getServoChannel(ChannelId.kChannelId1);

        configureServos();
        servoHub.configure(config, ResetMode.kResetSafeParameters);
    }

    private void configureServos() {
        config.channel0.pulseRange(1000, 1500, 2000);
        config.channel1.pulseRange(1000, 1500, 2000);

        channel0.setPowered(true);
        channel1.setPowered(true);

        channel0.setEnabled(true);
        channel1.setEnabled(true);

        channel0.setPulseWidth(Constants.servoDefaultPosition);
        channel1.setPulseWidth(Constants.servoDefaultPosition);

        servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 20000);

        servoHub.configure(config, ResetMode.kResetSafeParameters);
    }

    // Between 1000 and 2000
    public void setPosition(int position) {

        setPosition = MathUtil.clamp(position, 1000, 2000);

        System.out.println("SET POSITION " + position);

        setPosition = position;

        channel0.setPulseWidth(setPosition);
    };

    public void stop() {
    };

    public int getID() {
        return canID;
    };

    @Override
    public void periodic() {
        channel0.setPulseWidth(setPosition);
        channel1.setPulseWidth(setPosition);
    }
}