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
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.control.Constants;

public class ShooterManualCommand extends Command {
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private TurretSubsystem turret;

    public ShooterManualCommand(SubsystemCollection subsystemCollection) {
        shooter = subsystemCollection.getShooterSubsystem();
        hood = subsystemCollection.getHoodSubsystem();
        turret = subsystemCollection.getTurretSubsystem();

        addRequirements(shooter, hood);
        if (turret != null) {
            addRequirements(turret);
        }
    }

    @Override
    public void initialize() {
        if (turret != null) {
            turret.setAimMode(TurretAimMode.MANUAL);
            turret.setTargetAngleRadians(Constants.turretManualShootAngleDegrees.getRadians());
        }
    }

    @Override
    public void execute() {
        shooter.runRPM(Constants.SHOOTER_CLOSE_RPM);
        hood.setExtendoPosition(Constants.HOOD_CLOSE_EXTENDO_POSITION);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
        if (turret != null) {
            turret.setAimMode(TurretAimMode.AUTO);
        }
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until
        // interrupted.
        return false;
    }
}
