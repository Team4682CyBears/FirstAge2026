package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorsSubsystem;
import frc.robot.subsystems.MotorSubsystem;


public class DetectSpinningFlagCommand extends Command {

    SensorsSubsystem sensorsSubsystem;
    MotorSubsystem motorSubsystem;

    public DetectSpinningFlagCommand(SensorsSubsystem sensorSubsystem, MotorSubsystem motorSubsystem) {

        this.sensorsSubsystem = sensorSubsystem;
        this.motorSubsystem = motorSubsystem;

    };

    @Override
    public void initialize(){
    }

    @Override
    public void execute(){
    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
    }

}
