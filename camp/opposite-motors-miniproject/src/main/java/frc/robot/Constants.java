// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  // SparkFlex motor CAN IDs
  public static final int motor1CanID = 1;
  public static final int motor2CanID = 2;
  
  // Kraken motor CAN IDs
  public static final int krakenMotor1CanID = 3;
  public static final int krakenMotor2CanID = 4;

  // Rev Servo CAN IDs
  public static final int servoHubCanID = 37;

  // Rev Servo Left-Right Positions
  public static double servoLeftPosition = 500;
  public static double servoRightPosition = 2500;
  public static final double servoDefaultPosition = 1000;

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
