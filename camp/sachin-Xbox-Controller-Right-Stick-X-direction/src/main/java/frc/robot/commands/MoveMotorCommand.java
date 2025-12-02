// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: MoveMotorCommand.java
// Intent: Command to move motor based on joystick input
// ************************************************************

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Command that moves a motor based on joystick input from a DoubleSupplier.
 * The command continuously reads joystick input and applies it to the motor.
 * The command never finishes on its own and will run until interrupted.
 */
public class MoveMotorCommand extends Command {
  private final MotorSubsystem motorSubsystem;
  private final DoubleSupplier joystickInputSupplier;

  /**
   * Constructs a MoveMotorCommand.
   *
   * @param motorSubsystem the MotorSubsystem to control
   * @param joystickInputSupplier a DoubleSupplier that provides joystick input (-1.0 to 1.0)
   */
  public MoveMotorCommand(MotorSubsystem motorSubsystem, DoubleSupplier joystickInputSupplier) {
    this.motorSubsystem = motorSubsystem;
    this.joystickInputSupplier = joystickInputSupplier;
    addRequirements(motorSubsystem);
  }

  /**
   * Initializes the command. Called when the command is initially scheduled.
   */
  @Override
  public void initialize() {
    // No initialization needed
  }

  /**
   * Executes the command. Called every time the scheduler runs while the command is scheduled.
   * Reads joystick input and applies it to the motor.
   */
  @Override
  public void execute() {
    motorSubsystem.setMotorSpeed(joystickInputSupplier.getAsDouble());
  }

  /**
   * Called once the command ends or is interrupted.
   * Stops the motor when the command ends.
   *
   * @param interrupted whether the command was interrupted
   */
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.setMotorSpeed(0);
  }

  /**
   * Returns whether this command has finished.
   * This command never finishes on its own.
   *
   * @return false - command never finishes
   */
  @Override
  public boolean isFinished() {
    return false;
  }
}