// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveMotorCommand;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(MotorSubsystem subsystem) {
    CommandXboxController controller = new CommandXboxController(OperatorConstants.kDriverControllerPort);
    return Commands.sequence(subsystem.exampleMethodCommand(), new MoveMotorCommand(subsystem, () -> controller.getRightX()));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
