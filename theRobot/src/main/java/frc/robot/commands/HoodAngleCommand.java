package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HoodSubsystem;

public class HoodAngleCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public HoodAngleCommand(HoodSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void initialize() {
        this.hoodSubsystem.setAnglePosition((int) SmartDashboard.getNumber("Hood Angle", 1000));
        this.hoodSubsystem.setExtendoPosition((int) SmartDashboard.getNumber("Hood Extendo", 1000));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
