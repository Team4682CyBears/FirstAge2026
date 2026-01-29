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
    private boolean isEnabled = true;

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

        channel0.setPowered(isEnabled);
        channel1.setPowered(isEnabled);

        channel0.setEnabled(isEnabled);
        channel1.setEnabled(isEnabled);

        channel0.setPulseWidth(Constants.servoDefaultPosition);
        channel1.setPulseWidth(Constants.servoDefaultPosition);

        servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 20000);

        servoHub.configure(config, ResetMode.kResetSafeParameters);
    }

    // Between 1000 and 2000
    public void setPosition(int position) {
        // configureServos();

        setPosition = MathUtil.clamp(position, 1000, 2000);

        System.out.println("SET POSITION " + position);

        setPosition = position;

        channel0.setPulseWidth(setPosition);
    };

    public void stop() {
        // isEnabled = false;
        if (!isEnabled) {
            channel0.setPowered(isEnabled);
            channel1.setPowered(isEnabled);
            channel0.setEnabled(isEnabled);
            channel1.setEnabled(isEnabled);
        }
    };

    public int getID() {
        return canID;
    };

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Real Motor RPM (ID: %d)".formatted(canID), left);
        if (isEnabled) {
            // System.out.println("Run to position in periodic " + setPosition);
            channel0.setPulseWidth(setPosition);
            channel1.setPulseWidth(setPosition);
            // System.out.println("Set Position CH0 " + channel0.getPulseWidth());
            // System.out.println("Set Position CH1 " + channel1.getPulseWidth());
        }

    }
}