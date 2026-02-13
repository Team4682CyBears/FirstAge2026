package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SensorsSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.MotorSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class DetectSpinningFlagCommand extends Command {

    SensorsSubsystem sensorsSubsystem;
    MotorSubsystem motorSubsystem;
    Timer stopwatch;
    boolean experimentRunning;

    public DetectSpinningFlagCommand(SensorsSubsystem sensorsSubsystem, MotorSubsystem motorSubsystem) {

        this.sensorsSubsystem = sensorsSubsystem;
        this.motorSubsystem = motorSubsystem;

        addRequirements(sensorsSubsystem);
        addRequirements(motorSubsystem);
    };

    @Override
    public void initialize(){

        experimentRunning = false;
        motorSubsystem.setSpeed(); 
        this.stopwatch.reset(); 
    }

    @Override
    public void execute(){

        if (experimentRunning == false && motorSubsystem.getRPM() >= Constants.motorSpeed) {
            System.out.println("Motor has reached set speed");
            experimentRunning = true; 

            stopwatch.start();
            cycles += 1;

        }


    }

    @Override
    public boolean isFinished(){
        return false;
    }
    
    @Override
    public void end(boolean interrupted){
    }

}
