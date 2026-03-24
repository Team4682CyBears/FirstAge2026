// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShooterDefaultCommand.java
// Intent: Default command to handle shooter rev behavior
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends Command {
  private final SubsystemCollection subsystems;
  private final ShooterSubsystem shooter;

  public ShooterDefaultCommand(SubsystemCollection subsystems) {
    this.subsystems = subsystems;
    this.shooter = subsystems.getShooterSubsystem();
    addRequirements(shooter);
  }

  @Override
  public void execute() {
    if (!subsystems.isShooterAimerAvailable()) {
      shooter.stop();
      return;
    }

    ShooterAimer aimer = subsystems.getShooterAimer();
    if (aimer.getShouldRevShooter()) {
      shooter.runRPM(Constants.SHOOTER_CLOSE_RPM);
    } else {
      shooter.stop();
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
