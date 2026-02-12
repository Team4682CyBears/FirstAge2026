// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: DriveTimeCommand.java
// Intent: Forms a command to drive the wheels according to input parameters (encoder dead reckoning and accelormeter for rotation).
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTimeCommand extends Command {
  private DrivetrainSubsystem drivetrain;
  private Timer timer = new Timer();
  private boolean done = false;
  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
  private Boolean fieldRelativeMode = false;
  private double durationSecondsValue = 0.0;

  /**
   * Creates a new driveCommand.
   * 
   * @param drivetrainSubsystem - the drive train subsystem
   * @param x                   - the x velocity
   * @param y                   - the y velocity
   */
  public DriveTimeCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      ChassisSpeeds chassisSpeeds,
      double durationSeconds,
      Boolean fieldRelativeMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);
    this.chassisSpeeds = chassisSpeeds;
    this.fieldRelativeMode = fieldRelativeMode;

    durationSecondsValue = durationSeconds;
  }

  /**
   * Create a new robot-centric drive time command.
   * 
   * @param drivetrainSubsystem
   * @param chassisSpeeds
   * @param durationSeconds
   */
  public DriveTimeCommand(
      DrivetrainSubsystem drivetrainSubsystem,
      ChassisSpeeds chassisSpeeds,
      double durationSeconds) {
    this(drivetrainSubsystem, chassisSpeeds, durationSeconds, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.driveFieldCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
    timer.reset();
    timer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (fieldRelativeMode) {
      drivetrain.driveFieldCentric(chassisSpeeds);
    } else {
      drivetrain.driveRobotCentric(chassisSpeeds);
    }
    if (timer.hasElapsed(this.durationSecondsValue)) {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveFieldCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
    if (interrupted) {
      done = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}