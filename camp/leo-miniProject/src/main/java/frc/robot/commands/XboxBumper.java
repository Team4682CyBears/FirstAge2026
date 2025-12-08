// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: XboxBumper.java
// Intent: Xbox bumpers
// ************************************************************
package frc.robot.commands;
import frc.robot.subsystems.TalonMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class XboxBumper extends Command {
    private final TalonMotorSubsystem motorSubsystem;
    private final double motorSpeed;



    //
  public XboxBumper(TalonMotorSubsystem subsystem, double motorSpeed) {

    this.motorSubsystem = subsystem;
    this.motorSpeed = motorSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motorSubsystem.setMotorSpeed(motorSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    motorSubsystem.stopMotor();
  
  }
  // Returns true when the command should end.
  
  @Override
  // when button is pressed it will stop the command
  public boolean isFinished() {
    return false;
  }
}
