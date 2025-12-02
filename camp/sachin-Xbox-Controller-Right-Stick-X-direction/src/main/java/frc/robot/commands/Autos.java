// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: Autos.java
// Intent: Autonomous command factory
// ************************************************************

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class contains factory methods for autonomous commands.
 * This is a utility class and should not be instantiated.
 */
public final class Autos {
  
  /**
   * Creates an example autonomous command.
   *
   * @param motorSubsystem the MotorSubsystem to use
   * @return an autonomous command
   */
  public static Command exampleAuto(MotorSubsystem motorSubsystem) {
    CommandXboxController controller = 
        new CommandXboxController(OperatorConstants.kDriverControllerPort);
    return Commands.sequence(
        motorSubsystem.exampleMethodCommand(), 
        new MoveMotorCommand(motorSubsystem, () -> controller.getRightX()));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}