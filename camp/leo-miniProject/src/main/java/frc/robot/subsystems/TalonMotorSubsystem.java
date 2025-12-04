// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: TalonMotorSubsystem.java
// Intent: Talon motors
// ************************************************************


package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;

public class TalonMotorSubsystem extends SubsystemBase {

  private TalonFX motor;
  private final DutyCycleOut eeDutyCycle = new DutyCycleOut(0.0);
  private double motorSpeed = 0.0;

  // contstructer method that takes in a int can id and sets motor to a new TalonFX set to the imput of canId
  public TalonMotorSubsystem(int canId ) {
    this.motor = new TalonFX(canId);
    configureMotor();
  }

  @Override
  public void periodic(){
    if(motorSpeed == 0){
      motor.stopMotor();
    } else {
      eeDutyCycle.withOutput(MathUtil.clamp(motorSpeed, -1.0 , 1.0));
      motor.setControl(eeDutyCycle);
    }
  }
  
  // sets the motor speed to the double motor speed and the motor speed is set in th periodic which checks eveyr ticks
  public void setMotorSpeed(double motorSpeed){
    this.motorSpeed = motorSpeed;
  }


  public void stopMotor(){
    System.out.println("Stopping Motor");
    motor.stopMotor();
  }

  private void configureMotor(){
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

  
}
