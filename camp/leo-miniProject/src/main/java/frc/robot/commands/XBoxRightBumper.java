// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: XboxRightBumper.java
// Intent: Xbox right bumpers
// ************************************************************
package frc.robot.commands;

import frc.robot.Constants;

import frc.robot.subsystems.TalonMotorSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class XBoxRightBumper extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TalonMotorSubsystem m_subsystem;

  //creates a new xbox right bumper

  public XBoxRightBumper(TalonMotorSubsystem subsystem) {
    this.m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_subsystem.setMotorSpeed(Constants.motorSpeed);
    System.out.println("Setting motorSpeed to: " + Constants.motorSpeed );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_subsystem.setMotorSpeed(0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
