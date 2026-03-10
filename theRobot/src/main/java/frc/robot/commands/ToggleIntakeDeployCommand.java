// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ToggleIntakeDeployCommand.java
// Intent: command to deploy and retract the intake
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.control.IntakeWristMode;

/**
 * Toggle the intake wrist between deployed and retracted.
 * When deploying, also start the intake roller (RPM read from SmartDashboard "Intake RPM").
 * When retracting, stop the roller.
 */
public class ToggleIntakeDeployCommand extends InstantCommand {

    private final IntakeWristSubsystem wrist;
    private final IntakeRollerSubsystem roller;

    public ToggleIntakeDeployCommand(IntakeWristSubsystem wrist, IntakeRollerSubsystem roller) {
        this.wrist = wrist;
        this.roller = roller;
        addRequirements(wrist, roller);
    }

    @Override
    public void initialize() {
        IntakeWristMode intakeWristMode = wrist.getMode();
        if (intakeWristMode == IntakeWristMode.RETRACTED) {
            // currently retracted -> deploy + start roller
            wrist.setMode(IntakeWristMode.DEPLOYED);
            // TODO make this a constant after testing complete
            double rpm = SmartDashboard.getNumber("Intake RPM", 5000);
            roller.runRPM(rpm);
        } else {
            // currently deployed -> retract + stop roller
            wrist.setMode(IntakeWristMode.RETRACTED);
            roller.stop();
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
