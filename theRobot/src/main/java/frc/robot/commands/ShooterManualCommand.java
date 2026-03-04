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
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.SwerveYawMode;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterManualCommand extends Command {
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private KickerSubsystem kicker;
    private DrivetrainSubsystem drivetrain;
    private ShooterAimer aimer;

    public ShooterManualCommand(SubsystemCollection subsystemCollection) {
        shooter = subsystemCollection.getShooterSubsystem();
        hood = subsystemCollection.getHoodSubsystem();
        kicker = subsystemCollection.getKickerSubsystem();
        drivetrain = subsystemCollection.getDriveTrainSubsystem();

        addRequirements(shooter, hood, kicker);
    }

    @Override
    public void execute() {
        shooter.runRPM(aimer.minShooterSpeed());
        kicker.runRPM(aimer.minKickerSpeed());
        hood.setExtendoPosition(0.0);
        drivetrain.setSwerveYawMode(SwerveYawMode.JOYSTICK);
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
        this.kicker.stop();
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until
        // interrupted.
        return false;
    }
}
