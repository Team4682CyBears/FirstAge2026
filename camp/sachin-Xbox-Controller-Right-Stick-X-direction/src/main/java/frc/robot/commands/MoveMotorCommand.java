// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//use this as the command class to execute the code to move the motor

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** An example command that uses an example subsystem. */
public class MoveMotorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MotorSubsystem m_subsystem;
  private final DoubleSupplier joystickInput;

  /**
   * Creates a new MoveMotor.
   *
   * @param subsystem The subsystem used by this command.
   */
  public MoveMotorCommand(MotorSubsystem subsystem, DoubleSupplier joystickInput) {
    m_subsystem = subsystem;
    this.joystickInput = joystickInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.

  
  @Override
  public void execute() {
    m_subsystem.setMotorSpeed(joystickInput.getAsDouble()); //check this
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorSpeed(0);
  }
  @Override
  public boolean isFinished(){
    return false;
  }
}
