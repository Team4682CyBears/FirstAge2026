package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWristSubsystem;

public class IntakeWristAngleCommand extends Command{
    private final IntakeWristSubsystem intakeWristSubsystem;
    private final IntSupplier extendoSupplier;

    /** 
     * @param intakeWristSubsystem
     * @param extendoSupplier
     */

     public IntakeWristAngleCommand(IntakeWristSubsystem intakeWristSubsystem, IntSupplier extendoSupplier) {
        this.intakeWristSubsystem = intakeWristSubsystem;
        this.extendoSupplier= extendoSupplier;
        addRequirements(intakeWristSubsystem);
     }

     public void initialize(){
        this.intakeWristSubsystem.setPosition(extendoSupplier.getAsInt());
     }

     @Override
     public void end(boolean interrupted){

     }

     @Override
     public boolean isFinished(){
        return true;
     }

   

}