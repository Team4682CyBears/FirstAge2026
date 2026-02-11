// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: RobotContainer.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Implementation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.DetectSpinningFlagCommand;
import frc.robot.commands.OldExperimentCommand;
import frc.robot.commands.SpinMotorCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.MotorSubsystem;
import frc.robot.subsystems.SensorsSubsystem;
import au.grapplerobotics.CanBridge;

/**
 * This class is where the bulk of the robot should be declared. 
 */
public class RobotContainer {

  boolean spinnerEnabled = true;
  boolean CTREEnabled = true;
  boolean PWFEnabled = true;
  boolean LaserEnabled = true;
  double motorSpeed = Constants.motorSpeed;

  //private final Implementation tof = new Implementation(CTREEnabled, PWFEnabled, LaserEnabled, spinnerEnabled);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // Define Motor Speed in RPM
    // double motorSpeed = SmartDashboard.getNumber(null, 0);

    SensorsSubsystem sensorSubsystem = new SensorsSubsystem(CTREEnabled, PWFEnabled, LaserEnabled);
    MotorSubsystem motorSubsystem = new MotorSubsystem(Constants.motorCanID); // initiazlie motor
    new SpinMotorCommand(motorSubsystem); // start motor
    new DetectSpinningFlagCommand(sensorSubsystem, motorSubsystem);

    
    //new OldExperimentCommand(this.motorSpeed, sensorSubsystem, 20, spinnerEnabled);
    //SmartDashboard.putData("Run Experiment Command", detectSpinningFlagCommand);
    // TO:DO debug code for the laserCAN. REMOVE once the LaserCAN is configured.
    //CanBridge.runTCP();
    // .setDefaultCommand(runExperimentCommand);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}