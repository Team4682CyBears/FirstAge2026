// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NeoSubsystem extends SubsystemBase {

    SparkMax neoMotor;

    /** Creates a new NeoSubsystem. */
    public NeoSubsystem(int canID) {
        this.neoMotor = new SparkMax(canID, MotorType.kBrushless);

        neoMotor.set(0.0);

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);

        neoMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void setSpeed(double speed) {
        neoMotor.set(Math.max(Math.min(speed, 1), -1));
    }

    public void stop() {
        neoMotor.stopMotor();
    }
}
