// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: IntakeWristAngleCommand.java
// Intent: command to set intake wrist position
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeWristSubsystem;

/**
 * A class to command the intake wrist to a position based
 * on a double supplier
 */
public class IntakeWristAngleCommand extends Command{
    private final IntakeWristSubsystem intakeWristSubsystem;
    private final DoubleSupplier extendoSupplier;

    /** 
     * @param intakeWristSubsystem
     * @param extendoSupplier
     */
     public IntakeWristAngleCommand(IntakeWristSubsystem intakeWristSubsystem, DoubleSupplier extendoSupplier) {
        this.intakeWristSubsystem = intakeWristSubsystem;
        this.extendoSupplier= extendoSupplier;
        addRequirements(intakeWristSubsystem);
     }

     /**
      * called when the command starts
      */
     public void initialize(){
        this.intakeWristSubsystem.setPosition(extendoSupplier.getAsDouble());
     }

     /**
      * determines when the command is finished
      */
     @Override
     public boolean isFinished(){
        return true;
     }

   

}