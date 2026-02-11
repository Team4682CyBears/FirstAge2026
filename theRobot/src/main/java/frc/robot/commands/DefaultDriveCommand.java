// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: DefaultDriveCommand.java
// Intent: Forms a command to drive the robot. We ALWAYS use field oriented drive.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.SwerveYawMode;
import frc.robot.subsystems.DrivetrainSubsystem;

import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends Command {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DefaultDriveCommand(DrivetrainSubsystem drivetrainSubsystem,
            DoubleSupplier translationXSupplier,
            DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        commandedChassisSpeeds = new ChassisSpeeds(
                m_translationXSupplier.getAsDouble(),
                m_translationYSupplier.getAsDouble(),
                m_rotationSupplier.getAsDouble());
        if (m_drivetrainSubsystem.getSwerveYawMode() == SwerveYawMode.JOYSTICK){
            m_drivetrainSubsystem.driveFieldCentric(commandedChassisSpeeds);
        } else if (m_drivetrainSubsystem.getSwerveYawMode() == SwerveYawMode.AUTO){
            m_drivetrainSubsystem.driveFieldCentricShooting(commandedChassisSpeeds);
        }
        //else do nothing
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.driveFieldCentric(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}