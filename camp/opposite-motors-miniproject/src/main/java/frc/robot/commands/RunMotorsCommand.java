// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SparkFlexMotorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunMotorsCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final SparkFlexMotorSubsystem motor1;
  private final SparkFlexMotorSubsystem motor2;

  public RunMotorsCommand(SparkFlexMotorSubsystem motor1, SparkFlexMotorSubsystem motor2) {
    this.motor1 = motor1;
    this.motor2 = motor2;

    addRequirements(motor1, motor2);
  }

  @Override
  public void execute() {
    motor1.setTargetRPM(SmartDashboard.getNumber("Motor Target RPM (ID: %d)".formatted(motor1.getID()), 0));
    motor2.setTargetRPM(SmartDashboard.getNumber("Motor Target RPM (ID: %d)".formatted(motor2.getID()), 0));
  }

  @Override
  public void end(boolean interrupted) {
    motor1.stop();
    motor2.stop();
  }

  @Override
  public boolean isFinished() {
    return false;

  }
}
