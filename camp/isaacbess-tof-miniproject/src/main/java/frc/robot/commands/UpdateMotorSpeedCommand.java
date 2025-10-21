// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.common.ToFSensor;
import frc.robot.subsystems.NeoSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

// Simple command: set motor speed from ToF reading (speed = 5 - inches).
public class UpdateMotorSpeedCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

    // Motor subsystem
    private final NeoSubsystem motorSubsystem;

    // ToF sensor (getRangeInches returns inches)
    private final ToFSensor tofSensor;

    // Create command with subsystem and sensor
    public UpdateMotorSpeedCommand(NeoSubsystem motorSubsystem, ToFSensor tofSensor) {
        // Store references to the subsystem and sensor.
        this.motorSubsystem = motorSubsystem;
        this.tofSensor = tofSensor;

        // Declare subsystem dependencies for the scheduler.
        addRequirements(motorSubsystem);
    }

    // Stop motor when starting
    @Override
    public void initialize() {
        // Ensure motor is stopped on start.
        this.motorSubsystem.stop();
    }

    // Each loop: read inches, compute speed, set motor
    @Override
    public void execute() {
        SmartDashboard.putNumber("ToF Range", this.tofSensor.getRangeInches());
        double speed = Math.max(Math.min(1 - this.tofSensor.getRangeInches() / 7, 1), 0);
        this.motorSubsystem.setSpeed(speed * (3.0 / 4.0));
    }

    // Stop motor when ending
    @Override
    public void end(boolean interrupted) {
        this.motorSubsystem.stop();
    }

    // Never finishes on its own
    @Override
    public boolean isFinished() {
        return false;
    }
}
