// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: IntakeWristManualCommand.java
// Intent: command to handle manually moving the intake wrist
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWristSubsystem;

public class IntakeWristManualCommand extends Command {
    // Grabbed from IntakeWristSubsystem so it is in one place. Maybe should be
    // moved to Constants
    private static final double intakeWristForwardVoltage = IntakeWristSubsystem.intakeWristForwardVoltage;
    private static final double intakeWristReverseVoltage = IntakeWristSubsystem.intakeWristReverseVoltage;

    private static final double joystickDeadband = 0.1;

    private final IntakeWristSubsystem wristSubsystem;
    private final DoubleSupplier joystickSupplier;

    public IntakeWristManualCommand(IntakeWristSubsystem wristSubsystem, DoubleSupplier joystickSupplier) {
        this.wristSubsystem = wristSubsystem;
        this.joystickSupplier = joystickSupplier;
        addRequirements(wristSubsystem);
    }

    @Override
    public void execute() {
        double value = joystickSupplier.getAsDouble();
        if (Math.abs(value) < joystickDeadband) {
            wristSubsystem.stop();
        } else if (value > 0) {
            wristSubsystem.runVoltage(intakeWristForwardVoltage);
        } else {
            wristSubsystem.runVoltage(intakeWristReverseVoltage);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.stop();
    }
}
