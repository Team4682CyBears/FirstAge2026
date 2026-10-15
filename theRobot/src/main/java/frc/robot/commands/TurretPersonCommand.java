// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ShooterManualCommand.java
// Intent: command to handle manual shooting
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.TurretAimMode;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.control.Constants;

public class TurretPersonCommand extends Command {
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private TurretSubsystem turret;
    private CameraSubsystem limelight;

    public TurretPersonCommand(SubsystemCollection subsystemCollection) {
        shooter = subsystemCollection.getShooterSubsystem();
        hood = subsystemCollection.getHoodSubsystem();
        turret = subsystemCollection.getTurretSubsystem();
        limelight = subsystemCollection.getCameraSubsystem();

        addRequirements(shooter, hood);
        if (turret != null) {
            addRequirements(turret);
        }
    }

    @Override
    public void initialize() {
        if (turret != null) {
            turret.setAimMode(TurretAimMode.MANUAL);
        }
    }

    @Override
    public void execute() {
        // shooter.runRPM(Constants.SHOOTER_CLOSE_RPM);
        // hood.setExtendoPosition(Constants.HOOD_CLOSE_EXTENDO_POSITION);
        if (turret != null && limelight.personDetected) {
            double cameraRayAngleDegrees = -limelight.personAngle + 90.0 + Constants.personAimCameraYawOffsetDegrees;
            double cameraRayAngleRadians = Math.toRadians(cameraRayAngleDegrees);

            double targetXFromCamera = Constants.personAimAssumedDistanceInches * Math.cos(cameraRayAngleRadians);
            double targetYFromCamera = Constants.personAimAssumedDistanceInches * Math.sin(cameraRayAngleRadians);

            double targetXFromTurret = Constants.personAimCameraOffsetXInches + targetXFromCamera;
            double targetYFromTurret = Constants.personAimCameraOffsetYInches + targetYFromCamera;

            double turretAngleDegrees = Math.toDegrees(Math.atan2(targetYFromTurret, targetXFromTurret));
            if (turretAngleDegrees < 0.0) {
                turretAngleDegrees += 360.0;
            }
            turret.setTargetAngleRadians(Math.toRadians(MathUtil.clamp(turretAngleDegrees, Constants.turretMinAngleDegrees, Constants.turretMaxAngleDegrees)));
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.shooter.stop();
        if (turret != null) {
            //turret.setAimMode(TurretAimMode.AUTO);
            turret.stop();
        }
    }

    @Override
    public boolean isFinished() {
        // this command runs on a while true trigger. So, it should not stop until
        // interrupted.
        return false;
    }
}
