// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.UpdateMotorSpeedCommand;
import frc.robot.common.ToFSensor;
import frc.robot.subsystems.NeoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    private final NeoSubsystem m_neoSubsystem = new NeoSubsystem(Constants.kNeoMotorPort);

    private final ToFSensor toFSensor;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        this.toFSensor = new ToFSensor(Constants.kToFSensorPort, 5.0, 0);
        m_neoSubsystem.setDefaultCommand(new UpdateMotorSpeedCommand(this.m_neoSubsystem, this.toFSensor));
    }

}
