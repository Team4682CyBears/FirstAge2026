// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.common.ToFSensor;
import frc.robot.subsystems.NeoSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class UpdateMotorSpeedCommand extends Command {
    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    private final NeoSubsystem motorSubsystem;

    private final ToFSensor tofSensor;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public UpdateMotorSpeedCommand(NeoSubsystem motorSubsystem, ToFSensor tofSensor) {
        this.motorSubsystem = motorSubsystem;
        this.tofSensor = tofSensor;

        addRequirements(motorSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        this.motorSubsystem.stop();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = 5 - this.tofSensor.getRangeInches();
        this.motorSubsystem.setSpeed(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.motorSubsystem.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
