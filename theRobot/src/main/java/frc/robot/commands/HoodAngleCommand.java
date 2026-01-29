package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterHoodServoSubsystem;

public class HoodAngleCommand extends Command {
    private final ShooterHoodServoSubsystem hoodSubsystem;

    public HoodAngleCommand(ShooterHoodServoSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        this.hoodSubsystem.setPosition((int) SmartDashboard.getNumber("Hood Extension", 1000));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
