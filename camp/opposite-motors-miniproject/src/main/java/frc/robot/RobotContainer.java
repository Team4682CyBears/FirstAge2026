// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.common.ShooterAngle;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RunMotorsCommand;
import frc.robot.commands.RunMotorsVoltsCommand;
import frc.robot.commands.SetServoPositionCommand;
import frc.robot.subsystems.SparkFlexMotorSubsystem;
import frc.robot.subsystems.KrakenMotorSubsystem;
import frc.robot.subsystems.ShooterAngleServoSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  // SparkFlex motors
  private final SparkFlexMotorSubsystem m_motor1 = new SparkFlexMotorSubsystem(Constants.motor1CanID, Constants.motor2CanID);
  private final ShooterAngleServoSubsystem servoHub = new ShooterAngleServoSubsystem(Constants.servoHubCanID);
  
  // // Kraken motors
  // private final KrakenMotorSubsystem m_krakenMotor1 = new KrakenMotorSubsystem(Constants.krakenMotor1CanID, false);
  // private final KrakenMotorSubsystem m_krakenMotor2 = new KrakenMotorSubsystem(Constants.krakenMotor2CanID, true);

  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  public RobotContainer() {
    // SparkFlex motors SmartDashboard entries
    SmartDashboard.putNumber("Motor Target RPM 1", 0);
    SmartDashboard.putNumber("Motor Target Volts 1", 0);
    
    // // Kraken motors SmartDashboard entries
    // SmartDashboard.putNumber("Motor Target RPM (ID: %d)".formatted(m_krakenMotor1.getID()), 0);
    // SmartDashboard.putNumber("Motor Target RPM (ID: %d)".formatted(m_krakenMotor2.getID()), 0);

    configureBindings();
  }
  
  private void configureBindings() {
    // Left trigger runs SparkFlex motors
    



    m_driverController.leftTrigger().whileTrue(new RunMotorsCommand(m_motor1));
    m_driverController.rightTrigger().whileTrue(new RunMotorsVoltsCommand(m_motor1));
    m_driverController.rightBumper().toggleOnTrue(new SetServoPositionCommand(servoHub, ShooterAngle.RIGHT));
    m_driverController.leftBumper().toggleOnTrue(new SetServoPositionCommand(servoHub, ShooterAngle.LEFT));
    // Right trigger runs Kraken motors
    // m_driverController.rightTrigger().whileTrue(new RunMotorsCommand(m_krakenMotor1, m_krakenMotor2));
  }
}
