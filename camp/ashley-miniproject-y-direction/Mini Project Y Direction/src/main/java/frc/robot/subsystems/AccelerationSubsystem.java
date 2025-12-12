// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AccelerationSubsystem extends SubsystemBase {
  private final SparkMax motor = new SparkMax(1, MotorType.kBrushless);
  /** Creates a new ExampleSubsystem. */
  public AccelerationSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command accelerateCommand(double controllerYValue) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          setMotorSpeed(controllerYValue);
        });
  }

  public void setMotorSpeed(double speed) {
    motor.set(speed); 
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
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
