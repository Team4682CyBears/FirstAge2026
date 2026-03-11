// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ToggleSpindexerKickerCommand.java
// Intent: Toggle spindexer and kicker on/off together
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.control.Constants;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexerSpinner;

/**
 * Toggle the spindexer and kicker on/off together.
 */
public class ToggleSpindexerKickerCommand extends InstantCommand {
    private final SpindexerSpinner spindexer;
    private final KickerSubsystem kicker;
    private final boolean enable;

    public ToggleSpindexerKickerCommand(SpindexerSpinner spindexer, KickerSubsystem kicker, boolean enable) {
        this.spindexer = spindexer;
        this.kicker = kicker;
        this.enable = enable;
        addRequirements(spindexer, kicker);
    }

    @Override
    public void initialize() {
        if (enable) {
            spindexer.runRPMWtihSensor();
            double rpm = SmartDashboard.getNumber("Kicker RPM", Constants.KICKER_RPM);
            kicker.runRPM(rpm);
        } else {
            spindexer.stop();
            kicker.stop();
        }
    }
}
