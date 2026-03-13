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

public class IntakeRollerManualCommand extends Command {
    private final IntakeRollerSubsystem intakeRollerSubsystem;

    public IntakeRollerManualCommand(IntakeRollerSubsystem intakeRollerSubsystem) {
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        addRequirements(intakeRollerSubsystem);
    }

    @Override
    public void execute() {
        double rpm = SmartDashboard.getNumber("Intake RPM", 5000);
        intakeRollerSubsystem.runRPM(rpm);
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
