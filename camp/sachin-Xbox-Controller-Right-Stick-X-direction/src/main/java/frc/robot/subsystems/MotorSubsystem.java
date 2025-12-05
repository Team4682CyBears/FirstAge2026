// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - FirstAge - 2026
// File: MotorSubsystem.java
// Intent: Subsystem to control a single TalonFX motor
// ************************************************************

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for controlling a single TalonFX motor with joystick input.
 * Handles motor configuration, speed control, and safety features.
 */
public class MotorSubsystem extends SubsystemBase {

  private final TalonFX motor = new TalonFX(Constants.talonFXCANID);

  /**
   * Constructs a MotorSubsystem and configures the motor.
   */
  public MotorSubsystem() {
    configureMotor();
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
 

  /**
   * Queries a boolean state of the subsystem.
   * @return The current state
   */
  

  
  private void configureMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
    config.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
    config.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;

    config.CurrentLimits.StatorCurrentLimit = Constants.motorStatorCurrentMaximumAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode response = motor.getConfigurator().apply(config);
    if (!response.isOK()) {
        System.out.println(
            "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
    }
  }
}