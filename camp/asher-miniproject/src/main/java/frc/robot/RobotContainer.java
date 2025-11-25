// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: RobotContainer.java
// Intent: Defines the structure of the robot
// ************************************************************

package frc.robot;

import frc.robot.subsystems.MotorSubsystem;
import frc.robot.commands.DefaultMotorCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class RobotContainer {
  private MotorSubsystem motorSubsystem = new MotorSubsystem(Constants.motorCanID, Constants.encoderCanID);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    motorSubsystem.setDefaultCommand(
      new DefaultMotorCommand(motorSubsystem, 0));
  }

  /**
   * Provides the command to be executed during the autonomous period.
   *
   * @return The command to run in autonomous mode. Currently, this returns
   *         an InstantCommand which performs no action.
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
