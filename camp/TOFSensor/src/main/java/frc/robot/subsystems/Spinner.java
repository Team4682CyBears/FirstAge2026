// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Spinner extends SubsystemBase {
    private CANSparkMax Motor;

    private SparkPIDController PidController;
    private RelativeEncoder encoder;
    private double kPLeft, kILeft, kDLeft, kIzLeft, kFFLeft, kMaxOutputLeft, kMinOutputLeft, maxRPMLeft, maxVelLeft, minVelLeft, maxAccLeft, allowedErrLeft;
    private boolean isLeftMotorInverted = false;
    private double motorReferencePosition = 0.0;
    private boolean motorsInitalizedForSmartMotion = false;
    private boolean movementWithinTolerance = false;
    
    /** Creates a new ExampleSubsystem. */
  public Spinner() {
    Motor = new CANSparkMax(Constants.motorCanId);

  }
  /**
   * Example command factory method.
   *
   * @return a command
   * 
   */

  public void spin(double speed){
    Motor.speed(speed);
  }
 

}
