package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.control.Constants;

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
        double currentPos = wrist.getHoodPosition();
        double mid = (Constants.intakeWristDeployedPositionRotations + Constants.intakeWristRetractedPositionRotations) / 2.0;
        if (currentPos < mid) {
            // currently retracted -> deploy + start roller
            wrist.setExtendoPosition(Constants.intakeWristDeployedPositionRotations);
            double rpm = SmartDashboard.getNumber("Intake RPM", 3000);
            roller.runRPM(rpm);
        } else {
            // currently deployed -> retract + stop roller
            wrist.setExtendoPosition(Constants.intakeWristRetractedPositionRotations);
            roller.stop();
        }
    }
}
