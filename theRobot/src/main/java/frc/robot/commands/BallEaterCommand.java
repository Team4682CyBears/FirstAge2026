// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShooterManualCommand.java
// Intent: command to handle manual shooting
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.TurretAimMode;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.control.Constants;

public class BallEaterCommand extends Command {
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;

    public BallEaterCommand(SubsystemCollection subsystemCollection) {
        shooter = subsystemCollection.getShooterSubsystem();
        hood = subsystemCollection.getHoodSubsystem();

        addRequirements(shooter, hood);
    }

    @Override
    public void initialize() {
       
    }

    @Override
    public void execute() {
        shooter.runRPM(800.0);
        hood.setExtendoPosition(Constants.hoodMaxPositionRotations);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until
        // interrupted.
        return false;
    }
}
