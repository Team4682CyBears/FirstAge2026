// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ServoSubsystem.java
// Intent: Abstract class defining methods of servo
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import frc.robot.common.ShooterAngle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class ServoSubsystem extends SubsystemBase {
  
  public abstract void setPosition(ShooterAngle position);
  
  public abstract void stop();
  
  public abstract int getID();
}
