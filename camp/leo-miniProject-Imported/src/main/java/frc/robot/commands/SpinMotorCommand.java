package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorSubsystem;

public class SpinMotorCommand extends Command {

    private final MotorSubsystem motorSubsystem;

    public SpinMotorCommand(MotorSubsystem motorSubsystem) {

        this.motorSubsystem = motorSubsystem;
    }

    @Override
    public void initialize() {
        motorSubsystem.setSpeed();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
