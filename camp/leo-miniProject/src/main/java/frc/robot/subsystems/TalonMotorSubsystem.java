// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.math.MathUtil;


import static java.lang.Math.max;
import static java.lang.Math.min;

public class TalonMotorSubsystem extends SubsystemBase {

  private TalonFX motor;
  private final DutyCycleOut eeDutyCycle = new DutyCycleOut(0.0);


  /** Creates a new ExampleSubsystem. */
  public TalonMotorSubsystem(int canId ) {

    this.motor = new TalonFX(canId);
    configureMotor();

  }


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
        System.out.println("TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
    }
}

  public void setMotorSpeed(double motorSpeed){
    if(motorSpeed == 0){
      motor.stopMotor();
    } else {
      eeDutyCycle.withOutput(MathUtil.clamp(motorSpeed, -1.0 , 1.0));
      motor.setControl(eeDutyCycle);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
