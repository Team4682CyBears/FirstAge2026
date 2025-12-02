// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: MotorSubsystem.java
// Intent: Subsystem to control a single TalonFX motor
// ************************************************************

package frc.robot.subsystems;

import frc.robot.Constants.OperatorConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling a single TalonFX motor with joystick input.
 * Handles motor configuration, speed control, and safety features.
 */
public class MotorSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(OperatorConstants.talonFXCANID);

  /**
   * Constructs a MotorSubsystem and configures the motor.
   */
  public MotorSubsystem() {
    motor.setNeutralMode(NeutralModeValue.Brake);
    motor.setInverted(false);
  }

  /**
   * Sets the motor speed.
   * @param speed Motor speed from -1.0 to 1.0
   */
  public void setMotorSpeed(double speed) {
    speed = Math.max(-1.0, Math.min(1.0, speed));
    motor.set(speed);
  }
  
  /**
   * Creates an example command.
   * @return A command that runs once
   */
  public Command exampleMethodCommand() {
    return runOnce(() -> {/* one-time action goes here */});
  }

  /**
   * Queries a boolean state of the subsystem.
   * @return The current state
   */
  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}