// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkFlexMotorSubsystem extends SubsystemBase {

  private final SparkFlex motor;
  private final SparkFlexConfig motorConfig;
  private final int canID;
  private final boolean inverted;
  // TODO: Use real free rpm from the motor specs
  private final double kSparkFlexFreeRPM = 6784.0;

  public SparkFlexMotorSubsystem(int canID, boolean inverted) {
    this.motor = new SparkFlex(canID, MotorType.kBrushless);
    this.motorConfig = new SparkFlexConfig();
    this.canID = canID;
    this.inverted = inverted;
  }

  public void setTargetRPM(double rpm) {
    double fraction = 0.0;
    if (Double.isFinite(rpm)) {
      fraction = rpm / kSparkFlexFreeRPM;
      if (fraction > 1.0) {
        fraction = 1.0;
      } else if (fraction < -1.0) {
        fraction = -1.0;
      }
    }
    motorConfig.inverted(inverted);
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.set(fraction);

  }

  public double getRPM() {
    try {
      return motor.getEncoder().getVelocity();
    } catch (Exception e) {
      return -1;
    }
  }

  public void stop() {
    motor.stopMotor();
  }

  public int getID() {
    return canID;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Real Motor RPM (ID: %d)".formatted(canID), getRPM());
  }
}
