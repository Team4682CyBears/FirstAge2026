// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: IntakeRollerManualCommand.java
// Intent: command to handle manual intake rolling
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.common.IntakeDirection;

public class IntakeRollerManualCommand extends Command {
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final IntakeDirection intakeDirection;

    public IntakeRollerManualCommand(IntakeRollerSubsystem intakeRollerSubsystem, IntakeDirection intakeDirection) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.intakeDirection = intakeDirection;
        addRequirements(intakeRollerSubsystem);
    }

    @Override
    public void execute() {
        double directionFactor = intakeDirection == IntakeDirection.INTAKE ? 1 : -1;
        double rpm = SmartDashboard.getNumber("Intake RPM", 5000);
        intakeRollerSubsystem.runRPM(rpm * directionFactor);
    }

    @Override
    public void end(boolean interrupted) {
        intakeRollerSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
