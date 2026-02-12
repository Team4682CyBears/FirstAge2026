package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.HardwareConstants;
import frc.robot.subsystems.HoodSubsystem;

public class AutoAimCommand extends Command {
    private final HoodSubsystem hoodSubsystem;

    public AutoAimCommand(HoodSubsystem hoodSubsystem) {
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(hoodSubsystem);
    }

    @Override
    public void execute() {
        this.hoodSubsystem
                .setAnglePosition((int) SmartDashboard.getNumber("Hood Angle", HardwareConstants.HOOD_MIN_EXT));
        this.hoodSubsystem
                .setExtendoPosition((int) SmartDashboard.getNumber("Hood Extendo", HardwareConstants.HOOD_MIN_EXT));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
