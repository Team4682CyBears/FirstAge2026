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

import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.servohub.config.ServoHubConfig;
import com.revrobotics.servohub.ServoHub;
import com.revrobotics.ResetMode;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.servohub.ServoChannel.ChannelId;
import frc.robot.control.Constants;
import frc.robot.control.HardwareConstants;

/**
 * Subsystem for controlling the shooter hood angle and extendo using REV ServoHub.
 */
public class HoodSubsystem extends SubsystemBase {
    private final ServoHubConfig config;
    private final ServoHub servoHub;
    private ServoChannel rightAngleServoChannel;
    private ServoChannel leftAngleServoChannel;
    private ServoChannel extendoServoChannel;
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
        this.rightAngleServoChannel = servoHub.getServoChannel(ChannelId.kChannelId0);
        this.leftAngleServoChannel = servoHub.getServoChannel(ChannelId.kChannelId1);
        this.extendoServoChannel = servoHub.getServoChannel(ChannelId.kChannelId2);

        anglePosition = HardwareConstants.HOOD_MIN_EXT;
        extendoPosition = HardwareConstants.HOOD_MIN_EXT;

        configureServos();
    }

    /**
     * Configures all servo channels with pulse ranges and default positions.
     */
    private void configureServos() {
        configureServoChannel(rightAngleServoChannel, config.channel0, HardwareConstants.HOOD_MIN_EXT, (HardwareConstants.HOOD_MIN_EXT+HardwareConstants.HOOD_MAX_EXT)/2, HardwareConstants.HOOD_MAX_EXT);
        configureServoChannel(leftAngleServoChannel, config.channel1, HardwareConstants.HOOD_MIN_EXT, (HardwareConstants.HOOD_MIN_EXT+HardwareConstants.HOOD_MAX_EXT)/2, HardwareConstants.HOOD_MAX_EXT);
        configureServoChannel(extendoServoChannel, config.channel2, HardwareConstants.HOOD_MIN_EXT, (HardwareConstants.HOOD_MIN_EXT+HardwareConstants.HOOD_MAX_EXT)/2, HardwareConstants.HOOD_MAX_EXT);

        servoHub.setBankPulsePeriod(ServoHub.Bank.kBank0_2, 20000);

        servoHub.configure(config, ResetMode.kResetSafeParameters);
    }

    /**
     * Configures a single servo channel with the given pulse range.
     *
     * @param channel the servo channel to configure
     * @param servoConfig the servo channel configuration
     * @param minPulse the minimum pulse width
     * @param midPulse the middle pulse width
     * @param maxPulse the maximum pulse width
     */
    public void configureServoChannel(ServoChannel channel, ServoChannelConfig servoConfig, int minPulse, int midPulse, int maxPulse) {
        servoConfig.pulseRange(minPulse, midPulse, maxPulse);
        channel.setPowered(true);
        channel.setEnabled(true);
        channel.setPulseWidth(Constants.servoDefaultPosition);
    }

    /**
     * Sets the angle position for the hood servos.
     *
     * @param position pulse width between 1000 and 2000
     */
    public void setAnglePosition(int position) {
        anglePosition = MathUtil.clamp(position, HardwareConstants.HOOD_MIN_EXT, HardwareConstants.HOOD_MAX_EXT);
    };

    /**
     * Sets the extendo position for the hood extendo servo.
     *
     * @param position pulse width between 1000 and 2000
     */
    public void setExtendoPosition(int position) {
        extendoPosition = MathUtil.clamp(position, HardwareConstants.HOOD_MIN_EXT, HardwareConstants.HOOD_MAX_EXT);

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
        rightAngleServoChannel.setPulseWidth(anglePosition);
        leftAngleServoChannel.setPulseWidth(anglePosition);
        extendoServoChannel.setPulseWidth(extendoPosition);
    }
}