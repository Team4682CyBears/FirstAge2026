// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: Robot container + binds
// ************************************************************

package frc.robot;


import frc.robot.commands.XboxBumper;
import frc.robot.subsystems.TalonMotorSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final TalonMotorSubsystem talonSubsystem;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController mDrivercontroller =
      new CommandXboxController(Constants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    talonSubsystem = new TalonMotorSubsystem(Constants.talonCanID);
    // Configure the trigger bindings
    configureBindings();
  }

  // config xbox buttons to run the motors
  
  private void configureBindings() {
    mDrivercontroller.b().onTrue(
      new InstantCommand(talonSubsystem::stopMotor, talonSubsystem)
    );
    mDrivercontroller.leftBumper().whileTrue(new XboxBumper(talonSubsystem, Constants.motorSpeed));
    mDrivercontroller.rightBumper().whileTrue(new XboxBumper(talonSubsystem, -Constants.motorSpeed));
  }


}
