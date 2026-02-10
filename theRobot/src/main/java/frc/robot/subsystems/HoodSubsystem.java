// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: HoodSubsystem.java
// Intent: Changes of the shooter angle
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

/**
 * Subsystem for controlling the shooter hood angle and extendo using REV ServoHub.
 */
public class HoodSubsystem extends SubsystemBase {
    private final ServoHubConfig config;
    private final ServoHub servoHub;
    private ServoChannel channel0;
    private ServoChannel channel1;
    private ServoChannel channel2;
    private int anglePosition;
    private int extendoPosition;

    /**
     * Constructs a new HoodSubsystem.
     *
     * @param canID the CAN ID of the ServoHub
     */
    public HoodSubsystem(int canID) {
        this.config = new ServoHubConfig();
        this.servoHub = new ServoHub(canID);
        this.channel0 = servoHub.getServoChannel(ChannelId.kChannelId0);
        this.channel1 = servoHub.getServoChannel(ChannelId.kChannelId1);
        this.channel2 = servoHub.getServoChannel(ChannelId.kChannelId2);

        anglePosition = Constants.HOOD_MIN_EXT;
        extendoPosition = Constants.HOOD_MIN_EXT;

        configureServos();
        servoHub.configure(config, ResetMode.kResetSafeParameters);
    }

    /**
     * Configures all servo channels with pulse ranges and default positions.
     */
    private void configureServos() {
        config.channel0.pulseRange(Constants.HOOD_MIN_EXT, (Constants.HOOD_MIN_EXT+Constants.HOOD_MAX_EXT)/2, Constants.HOOD_MAX_EXT);
        config.channel1.pulseRange(Constants.HOOD_MIN_EXT, (Constants.HOOD_MIN_EXT+Constants.HOOD_MAX_EXT)/2, Constants.HOOD_MAX_EXT);
        config.channel2.pulseRange(Constants.HOOD_MIN_EXT, (Constants.HOOD_MIN_EXT+Constants.HOOD_MAX_EXT)/2, Constants.HOOD_MAX_EXT);

        channel0.setPowered(true);
        channel1.setPowered(true);
        channel2.setPowered(true);

        channel0.setEnabled(true);
        channel1.setEnabled(true);
        channel2.setEnabled(true);

        channel0.setPulseWidth(Constants.servoDefaultPosition);
        channel1.setPulseWidth(Constants.servoDefaultPosition);
        channel2.setPulseWidth(Constants.servoDefaultPosition);

        servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 20000);

        servoHub.configure(config, ResetMode.kResetSafeParameters);
    }

    /**
     * Sets the angle position for the hood servos.
     *
     * @param position pulse width between 1000 and 2000
     */
    public void setAnglePosition(int position) {
        anglePosition = MathUtil.clamp(position, Constants.HOOD_MIN_EXT, Constants.HOOD_MAX_EXT);

        anglePosition = position;
    };

    /**
     * Sets the extendo position for the hood extendo servo.
     *
     * @param position pulse width between 1000 and 2000
     */
    public void setExtendoPosition(int position) {
        extendoPosition = MathUtil.clamp(position, Constants.HOOD_MIN_EXT, Constants.HOOD_MAX_EXT);

        extendoPosition = position;
    }

    /**
     * Returns the current angle position pulse width for the hood servos.
     * Useful for logging/testing.
     *
     * @return pulse width (1000-2000)
     */
    public int getAnglePosition() {
        return anglePosition;
    }

    /**
     * Returns the current extendo position pulse width for the hood extendo servo.
     *
     * @return pulse width (1000-2000)
     */
    public int getExtendoPosition() {
        return extendoPosition;
    }

    @Override
    public void periodic() {
        channel0.setPulseWidth(anglePosition);
        channel1.setPulseWidth(anglePosition);
        channel2.setPulseWidth(extendoPosition);
    }
}