// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: Xbox left bumpers
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



  // contstructer method that takes in a int can id and sets motor to a new TalonFX set to the imput of canId
  public TalonMotorSubsystem(int canId ) {

    this.motor = new TalonFX(canId);
    configureMotor();

  }




  
  // sets the motor speed to the double imput and clamps that from -1 <> 1 and if the motor speed set to 0 it stops the motor
  public void setMotorSpeed(double motorSpeed){
    if(motorSpeed == 0){
      motor.stopMotor();
      System.out.println("Stopping Motor");
    } else {
      eeDutyCycle.withOutput(MathUtil.clamp(motorSpeed, -1.0 , 1.0));
      motor.setControl(eeDutyCycle);
    }
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

  
}
