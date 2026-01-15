// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SparkFlexMotorSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class RunMotorsVoltsCommand extends Command {
  private final SparkFlexMotorSubsystem motor1;

  public RunMotorsVoltsCommand(SparkFlexMotorSubsystem motor1) {
    this.motor1 = motor1;

    addRequirements(motor1);
  }

  @Override
  public void execute() {
    motor1.runVoltage(SmartDashboard.getNumber("Motor Target Volts 1", 0));
  }

  @Override
  public void end(boolean interrupted) {
    motor1.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
