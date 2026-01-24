package frc.robot.commands;

import frc.robot.subsystems.ShooterAngleServoSubsystem;
import frc.robot.common.ShooterAngle;
import edu.wpi.first.wpilibj2.command.Command;

public class SetServoPositionCommand extends Command{
    private ShooterAngleServoSubsystem servoHub;
    private ShooterAngle angle;

    public SetServoPositionCommand(ShooterAngleServoSubsystem servoHub, ShooterAngle angle) {
        this.angle = angle;
        this.servoHub = servoHub;

        addRequirements(servoHub);
    }

    @Override
    public void execute() {
        
        //motor1.setTargetRPM(100);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
