// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunMotorsCommand;
import frc.robot.subsystems.SparkFlexMotorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  private final SparkFlexMotorSubsystem m_motor1 = new SparkFlexMotorSubsystem(Constants.motor1CanID, false);
  private final SparkFlexMotorSubsystem m_motor2 = new SparkFlexMotorSubsystem(Constants.motor2CanID, true);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  public RobotContainer() {
    SmartDashboard.putNumber("Motor Target RPM (ID: %d)".formatted(m_motor1.getID()), 0);
    SmartDashboard.putNumber("Motor Target RPM (ID: %d)".formatted(m_motor2.getID()), 0);

    configureBindings();
  }

  private void configureBindings() {
    m_driverController.leftTrigger().whileTrue(new RunMotorsCommand(m_motor1, m_motor2));
  }
}
