// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: RobotContainer.java
// Intent: Robot container for subsystems and commands
// ************************************************************



package frc.robot;

import frc.robot.Constants;
import frc.robot.commands.MoveMotorCommand;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final MotorSubsystem motorSubsystem = new MotorSubsystem();
  private final CommandXboxController driverController = 
      new CommandXboxController(Constants.kDriverControllerPort);

  /**
   * Constructs the RobotContainer and configures trigger->command mappings.
   */
  public RobotContainer() {
    configureBindings();
    motorSubsystem.setDefaultCommand(
        new MoveMotorCommand(motorSubsystem, () -> driverController.getRightX()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
   * constructor with an arbitrary predicate, or via the named factories in
   * {@link CommandXboxController}'s subclasses for Xbox or PS4
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Command Joystick}.
   */
  private void configureBindings() {
    // Add button bindings here
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}