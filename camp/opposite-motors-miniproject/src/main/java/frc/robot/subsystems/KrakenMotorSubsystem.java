// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class KrakenMotorSubsystem extends MotorSubsystem {

    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig;
    private final int canID;
    private final boolean inverted;
    private final double kKrakenFreeRPM = 6000.0;
    private final VelocityVoltage velocityControl;

    public KrakenMotorSubsystem(int canID, boolean inverted) {
        this.motor = new TalonFX(canID);
        this.motorConfig = new TalonFXConfiguration();
        this.canID = canID;
        this.inverted = inverted;
        this.velocityControl = new VelocityVoltage(0);

        configureMotor();
    }

    private void configureMotor() {
        motorConfig.MotorOutput.Inverted = inverted ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor.getConfigurator().apply(motorConfig);
    }

    public void setTargetRPM(double rpm) {
        if (!Double.isFinite(rpm)) {
            rpm = 0.0;
        }

        // Clamp RPM to reasonable values
        if (rpm > kKrakenFreeRPM) {
            rpm = kKrakenFreeRPM;
        } else if (rpm < -kKrakenFreeRPM) {
            rpm = -kKrakenFreeRPM;
        }

        // Convert RPM to rotations per second for Phoenix 6
        double rotationsPerSecond = rpm / 60.0;
        motor.setControl(velocityControl.withVelocity(rotationsPerSecond));
    }

    public double getRPM() {
        try {
            // Get velocity in rotations per second and convert to RPM
            return motor.getVelocity().getValueAsDouble() * 60.0;
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
