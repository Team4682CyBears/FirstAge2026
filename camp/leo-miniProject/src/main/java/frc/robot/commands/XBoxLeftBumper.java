// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: Xbox left bumpers
// ************************************************************
package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.subsystems.TalonMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class XBoxLeftBumper extends Command {
  private final TalonMotorSubsystem motorSubsystem;



  //creates a new xbox left bumper command
   
  public XBoxLeftBumper(TalonMotorSubsystem subsystem) {
    this.motorSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorSubsystem.setMotorSpeed(-Constants.motorSpeed);
    System.out.println("Setting motorSpeed to: " + Constants.motorSpeed );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.setMotorSpeed(0);
  
  }
  // Returns true when the command should end.
  @Override
  
  // when button is pressed it will stop the command
  public boolean isFinished() {
    return false;
  }
}
