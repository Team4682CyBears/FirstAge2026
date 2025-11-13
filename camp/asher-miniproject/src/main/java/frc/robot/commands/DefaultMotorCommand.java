// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier;

public class DefaultMotorCommand extends Command {
  private final MotorSubsystem motorSubsystem;
  private final DoubleSupplier encoderSupplier;

  private double targetAngle;

  private final double encoderDeadband = 2.0;
  

  public DefaultMotorCommand(MotorSubsystem motorSubsystem, double targetAngle) {
    this.motorSubsystem = motorSubsystem;
    addRequirements(motorSubsystem);
    this.encoderSupplier = () -> motorSubsystem.getEndodgerPositionDegrees();

    this.targetAngle = targetAngle;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderValue = encoderSupplier.getAsDouble();
    System.out.println(encoderValue);

    if (Math.abs(encoderValue - targetAngle) > encoderDeadband) {
      if (encoderValue < targetAngle) {
        motorSubsystem.setMotorSpeed(0.1); // Move forward
      } else {
        motorSubsystem.setMotorSpeed(-0.1); // Move backward
      }
    } else {
      motorSubsystem.setMotorSpeed(0.0); // Stop motor
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

}
