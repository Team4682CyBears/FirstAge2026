// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ClimberPositionCommand.java
// Intent: command to set climber position
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.ClimberSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberPositionCommand extends Command {

    ClimberSubsystem climberSubsystem;
    DoubleSupplier targetHeightSupplier;

    public ClimberPositionCommand(ClimberSubsystem climberSubsystem, DoubleSupplier targetHeightSupplier) {
        this.climberSubsystem = climberSubsystem;
        this.targetHeightSupplier = targetHeightSupplier;
        addRequirements(climberSubsystem);
    }

    @Override
    public void execute() {
        this.climberSubsystem.goToPosition(targetHeightSupplier.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return this.climberSubsystem.isClimberWithinTolerance(this.climberSubsystem.getTargetPosition());
    }
}
