// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: RobotContainer.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import frc.robot.commands.DefaultUpdateMotorSpeedCommand;
import frc.robot.common.ToFSensor;
import frc.robot.subsystems.NeoSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final NeoSubsystem neoSubsystem = new NeoSubsystem(Constants.kNeoMotorPort);

    private final ToFSensor toFSensor = new ToFSensor(Constants.kToFSensorPort, 5.0, 0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        neoSubsystem.setDefaultCommand(new DefaultUpdateMotorSpeedCommand(this.neoSubsystem, this.toFSensor));
    }

}
