package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooterSubsystem;

    public ShootCommand(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void execute() {
        this.shooterSubsystem.runRPM(SmartDashboard.getNumber("Shooter RPM", 0));
    }

    @Override
    public void end(boolean interrupted) {
        this.shooterSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
