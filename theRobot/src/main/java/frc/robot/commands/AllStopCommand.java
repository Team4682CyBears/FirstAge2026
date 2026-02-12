// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: AllStopCommand.java
// Intent: Forms a command to stop all subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.control.SubsystemCollection;

/**
 * Class to form a command to stop all subsystems
 */
// TODO - update this command
public class AllStopCommand extends Command {
    private final SubsystemCollection subsystems;

    /**
     * Constructor to cause all subsystems to halt movements
     * 
     * @param collection - the collection of subsystems
     */
    public AllStopCommand(SubsystemCollection collection) {
        subsystems = collection;
        // TODO add all subsystems this command relies on
        if (this.subsystems.isDriveTrainSubsystemAvailable()) {
            addRequirements(this.subsystems.getDriveTrainSubsystem());
        }
        if (this.subsystems.isShooterSubsystemAvailable()) {
            addRequirements(this.subsystems.getShooterSubsystem());
        }
        if (this.subsystems.isKickerSubsystemAvailable()) {
            addRequirements(this.subsystems.getKickerSubsystem());
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (this.subsystems.isDriveTrainSubsystemAvailable()) {
            this.subsystems.getDriveTrainSubsystem().driveFieldCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
        }
        if (this.subsystems.isShooterSubsystemAvailable()) {
            this.subsystems.getShooterSubsystem().stop();
        }
        if (this.subsystems.isKickerSubsystemAvailable()) {
            this.subsystems.getKickerSubsystem().stop();
        }
        CommandScheduler.getInstance().cancelAll();
        System.out.println("I CLEARED ALL COMMANDS!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");

        // TODO add stop commands for all other subsystems here.
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}