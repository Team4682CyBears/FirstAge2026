// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ToggleAutoAimMovingCommand.java
// Intent: Toggle auto-aim moving behavior
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.control.ShooterAimer;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Toggle auto-aim moving behavior on/off with state stored in drivetrain.
 */
public class ToggleAutoAimMovingCommand extends InstantCommand {
  private final DrivetrainSubsystem drivetrain;
  private final AutoAimMovingCommand autoAimMovingCommand;
  private final boolean enable;

  public ToggleAutoAimMovingCommand(SubsystemCollection subsystemCollection, ShooterAimer aimer, boolean enable) {
    this.drivetrain = subsystemCollection.getDriveTrainSubsystem();
    this.autoAimMovingCommand = new AutoAimMovingCommand(subsystemCollection, aimer);
    this.enable = enable;
  }

  @Override
  public void initialize() {
    drivetrain.setAutoAimMovingEnabled(enable);

    if (enable) {
      CommandScheduler.getInstance().schedule(autoAimMovingCommand);
    } else {
      CommandScheduler.getInstance().cancel(autoAimMovingCommand);
    }
  }
}
