// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ClimberVelocityCommand.java
// Intent: command to handle manually moving the climber
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberVelocityCommand extends Command {

    private static final double joystickDeadband = 0.1;

    private final ClimberSubsystem climberSubsystem;
    private final DoubleSupplier joystickSupplier;

    public ClimberVelocityCommand(ClimberSubsystem climberSubsystem, DoubleSupplier joystickSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.joystickSupplier = joystickSupplier;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        this.climberSubsystem.setManualMode(true);
        double value = joystickSupplier.getAsDouble();
        if (Math.abs(value) < joystickDeadband) {
            climberSubsystem.stop();
        } else if (value > 0) {
            climberSubsystem.runVoltage(-1.5);
        } else {
            climberSubsystem.runVoltage(1.5);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climberSubsystem.stop();
    }
}
