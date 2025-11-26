// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - First Age - 2026
// File: Spinner.java
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

/**The spinner class helps to create motor subsystems which allow the
* rest of the code to accsess the SparkMax motor controller which
* controls the neo motor.
*/
public class Spinner extends SubsystemBase {
    private SparkMax motor;
    private double speed;


    /**
     * Constructer function for a Spinner subsystem
     */
  public Spinner() {
    motor = new SparkMax(Constants.motorCanID, MotorType.kBrushless);

    // Configure motor to break when stopped
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake);

    // Apply config
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

    /**
   * publishes the current speed of the motor to the Smartdashboard
   */
  public void publishTelemetery(){
    SmartDashboard.putNumber(" Current Speed " , this.speed);
  } 

   /**
   * Takes in a speed parameter and sets the motor speed to that speed
   */
  public void spin(double speed){
    motor.set(speed);
    this.speed = speed;
  }
  


 

}
