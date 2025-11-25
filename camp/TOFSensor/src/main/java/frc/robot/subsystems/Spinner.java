// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Spinner extends SubsystemBase {
    private SparkMax motor;
  /* 
    private RelativeEncoder encoder;
    private double kPLeft, kILeft, kDLeft, kIzLeft, kFFLeft, kMaxOutputLeft, kMinOutputLeft, maxRPMLeft, maxVelLeft, minVelLeft, maxAccLeft, allowedErrLeft;
    private boolean isLeftMotorInverted = false;
    private double motorReferencePosition = 0.0;
    private boolean motorsInitalizedForSmartMotion = false;
    private boolean movementWithinTolerance = false;
    */
    /** Creates a new ExampleSubsystem. */
  public Spinner() {
    motor = new SparkMax(Constants.motorCanID, MotorType.kBrushless);

    // Configure motor to break when stopped
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    // Apply config
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
  /**
   * Example command factory method.
   *
   * @return a command
   * 
   */

  public void spin(double speed){
    motor.set(speed);
  }
 

}
