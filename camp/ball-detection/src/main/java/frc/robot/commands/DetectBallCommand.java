// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: CameraSubsystem.java
// Intent: command for ball dectection
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.BallDetectionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;

/** An example command that uses an example subsystem. */
public class DetectBallCommand extends Command {
  @SuppressWarnings("PMD.UnusedPrivateField")
  private final BallDetectionSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DetectBallCommand(BallDetectionSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimelightHelpers.setPipelineIndex("limelight", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< Updated upstream
    System.out.println(LimelightHelpers.getTX("limelight"));
=======
    SmartDashboard.putBoolean("Ball detected", BallDetectionSubsystem.ballDetected("limelight"));
    System.out.println(BallDetectionSubsystem.ballDetected("limelight"));
>>>>>>> Stashed changes
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
