// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: DefaultMotorCommand.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

/**
 * A command that controls a motor subsystem to move to a target angle.
 * The motor will adjust its speed based on the difference between the current
 * encoder position and the target angle, stopping when the difference is within
 * a specified deadband.
 */
public class DefaultMotorCommand extends Command {
  private MotorSubsystem motorSubsystem;
  private DoubleSupplier encoderSupplier;
  private double targetAngle;

  private final double encoderDeadbandDegrees = 2.0;

  /**
   * Constructs a new DefaultMotorCommand.
   *
   * @param motorSubsystem The motor subsystem to be controlled.
   * @param targetAngle The target angle in degrees that the motor should move to.
   */
  public DefaultMotorCommand(MotorSubsystem motorSubsystem, double targetAngle) {
    this.motorSubsystem = motorSubsystem;
    addRequirements(motorSubsystem);
    this.encoderSupplier = () -> motorSubsystem.getEndodgerPositionDegrees();
    this.targetAngle = targetAngle;
  }

  /**
   * Executes the command. Adjusts the motor speed to move the motor subsystem
   * towards the target angle. Stops the motor when the target angle is reached
   * within the deadband.
   */
  @Override
  public void execute() {
    double currentAngle = encoderSupplier.getAsDouble();
    double angleDifference = targetAngle - currentAngle;

    if (Math.abs(angleDifference) <= encoderDeadbandDegrees) {
      motorSubsystem.setMotorSpeed(0);
    } else {
      double motorSpeed = angleDifference > 0 ? 0.5 : -0.5;
      motorSubsystem.setMotorSpeed(motorSpeed);
    }
  }

  /**
   * Called once the command ends or is interrupted. Cleans up resources or
   * performs any necessary finalization.
   *
   * @param interrupted Whether the command was interrupted.
   */
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.setMotorSpeed(0);
  }
}
