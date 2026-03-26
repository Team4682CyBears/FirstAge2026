// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: KickerSpindexerAgitateCommand.java
// Intent: command to run kicker + spindexer while agitating intake wrist
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.IntakeWristSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.SpindexerSpinner;

/**
 * Command to run the kicker and spindexer together while agitating the intake wrist.
 */
public class KickerSpindexerAgitateCommand extends Command {
    private static final double defaultTogglePeriodSeconds = 0.5;

    private final KickerSubsystem kickerSubsystem;
    private final SpindexerSpinner spindexerSpinner;
    private final IntakeWristSubsystem intakeWristSubsystem;
    private final Timer timer = new Timer();
    private final double togglePeriodSeconds;

    private boolean wristIsDeployed = true;
    private double nextToggleTimeSeconds = defaultTogglePeriodSeconds;

    /**
     * Constructs a new KickerSpindexerAgitateCommand.
     *
     * @param kickerSubsystem
     * @param spindexerSpinner
     * @param intakeWristSubsystem
     * @param kickSpeedSupplier
     */
    public KickerSpindexerAgitateCommand(
            KickerSubsystem kickerSubsystem,
            SpindexerSpinner spindexerSpinner,
            IntakeWristSubsystem intakeWristSubsystem) {
        this(kickerSubsystem, spindexerSpinner, intakeWristSubsystem, defaultTogglePeriodSeconds);
    }

    /**
     * Constructs a new KickerSpindexerAgitateCommand with a custom toggle period.
     *
     * @param kickerSubsystem
     * @param spindexerSpinner
     * @param intakeWristSubsystem
     * @param kickSpeedSupplier
     * @param togglePeriodSeconds
     */
    public KickerSpindexerAgitateCommand(
            KickerSubsystem kickerSubsystem,
            SpindexerSpinner spindexerSpinner,
            IntakeWristSubsystem intakeWristSubsystem,
            double togglePeriodSeconds) {
        this.kickerSubsystem = kickerSubsystem;
        this.spindexerSpinner = spindexerSpinner;
        this.intakeWristSubsystem = intakeWristSubsystem;
        this.togglePeriodSeconds = togglePeriodSeconds;
        addRequirements(kickerSubsystem, spindexerSpinner, intakeWristSubsystem);
    }

    @Override
    public void initialize() {
    kickerSubsystem.runRPM(Constants.KICKER_RPM);
        spindexerSpinner.runRPMContinus();
        wristIsDeployed = true;
        intakeWristSubsystem.setPosition(Constants.intakeWristDeployedPositionRotations);
        timer.reset();
        timer.start();
    nextToggleTimeSeconds = togglePeriodSeconds;
    }

    @Override
    public void execute() {
        if (timer.hasElapsed(nextToggleTimeSeconds)) {
            wristIsDeployed = !wristIsDeployed;
            double targetPosition = wristIsDeployed
                    ? Constants.intakeWristDeployedPositionRotations
                    : Constants.intakeWristAgitateStowPositionRotations;
            intakeWristSubsystem.setPosition(targetPosition);
            timer.reset();
            timer.start();
        }
    }

    @Override
    public void end(boolean interrupted) {
        kickerSubsystem.stop();
        spindexerSpinner.stop();
        intakeWristSubsystem.setPosition(Constants.intakeWristDeployedPositionRotations);
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until interrupted.
        return false;
    }
}
