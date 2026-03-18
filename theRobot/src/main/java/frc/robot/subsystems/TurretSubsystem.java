// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: TurretSubsystem.java
// Intent: Control the turret aiming motor for the shooter.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.HardwareConstants;
import frc.robot.control.ShooterAimer;
import frc.robot.control.TurretAimMode;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretMotor;
    private final DrivetrainSubsystem drivetrain;

    private ShooterAimer shooterAimer;

    private final PositionVoltage positionController = new PositionVoltage(0.0);

    private TurretAimMode turretAimMode = TurretAimMode.AUTO;
    private double targetTurretAngleRadians = 0.0;

    private final double minTurretAngleRadians = Math.toRadians(Constants.turretMinAngleDegrees);
    private final double maxTurretAngleRadians = Math.toRadians(Constants.turretMaxAngleDegrees);

    // TODO tune with robot-on-carpet data
    private final Slot0Configs slot0Configs = new Slot0Configs()
            .withKP(Constants.turretPositionKp)
            .withKI(Constants.turretPositionKi)
            .withKD(Constants.turretPositionKd);

    public TurretSubsystem(int turretCanId, DrivetrainSubsystem drivetrain) {
        this.turretMotor = new TalonFX(turretCanId);
        this.drivetrain = drivetrain;
        configureMotor();
        zeroTurret();
    }

    public void setShooterAimer(ShooterAimer aimer) {
        this.shooterAimer = aimer;
    }

    public ShooterAimer getShooterAimer() {
        return shooterAimer;
    }

    public void setTurretAimMode(TurretAimMode mode) {
        if (mode == TurretAimMode.JOYSTICK) {
            targetTurretAngleRadians = getTurretAngleRadians();
        }
        turretAimMode = mode;
    }

    public TurretAimMode getTurretAimMode() {
        return turretAimMode;
    }

    public void setTurretAngleRadians(double turretAngleRadians) {
        targetTurretAngleRadians = MathUtil.clamp(turretAngleRadians, minTurretAngleRadians, maxTurretAngleRadians);
    }

    public double getTurretAngleRadians() {
        double motorRotations = turretMotor.getPosition().getValueAsDouble();
        double turretRotations = motorRotations / Constants.turretGearRatio;
        return turretRotations * 2.0 * Math.PI;
    }

    public Rotation2d getTurretFieldRotation() {
        return drivetrain.getGyroscopeRotation().plus(Rotation2d.fromRadians(getTurretAngleRadians()));
    }

    public void zeroTurret() {
        turretMotor.setPosition(0.0);
        targetTurretAngleRadians = 0.0;
    }

    @Override
    public void periodic() {
        if (turretAimMode == TurretAimMode.AUTO && shooterAimer != null) {
            Translation2d defaultTarget = getDefaultAimTarget();
            shooterAimer.setDefaultDesiredTarget(defaultTarget);
            shooterAimer.calculate();
            double desiredTurretAngle = computeDesiredTurretAngleRadians();
            setTurretAngleRadians(desiredTurretAngle);
        }

        double targetMotorRotations = radiansToMotorRotations(targetTurretAngleRadians);
        positionController.withPosition(targetMotorRotations);
        turretMotor.setControl(positionController);

        SmartDashboard.putNumber("TurretAngleDegrees", Math.toDegrees(getTurretAngleRadians()));
        SmartDashboard.putNumber("TurretTargetDegrees", Math.toDegrees(targetTurretAngleRadians));
    }

    private double computeDesiredTurretAngleRadians() {
        double robotYawRadians = drivetrain.getGyroscopeRotation().getRadians();
        double desiredFieldYawRadians = shooterAimer.getAutoYaw().getRadians();
        double desiredRelativeRadians = MathUtil.inputModulus(
                desiredFieldYawRadians - robotYawRadians,
                0.0,
                2.0 * Math.PI);
        return wrapAngleToTurretRange(desiredRelativeRadians);
    }

    private double wrapAngleToTurretRange(double desiredRelativeRadians) {
        double range = maxTurretAngleRadians - minTurretAngleRadians;
        if (range <= 0.0) {
            return minTurretAngleRadians;
        }

        double normalizedDesired = MathUtil.inputModulus(desiredRelativeRadians, 0.0, 2.0 * Math.PI);
        double normalizedMin = MathUtil.inputModulus(minTurretAngleRadians, 0.0, 2.0 * Math.PI);
        double normalizedMax = MathUtil.inputModulus(maxTurretAngleRadians, 0.0, 2.0 * Math.PI);

        boolean inRange = normalizedMin <= normalizedMax
                ? normalizedDesired >= normalizedMin && normalizedDesired <= normalizedMax
                : normalizedDesired >= normalizedMin || normalizedDesired <= normalizedMax;
        if (inRange) {
            return normalizedDesired;
        }

        double distanceToMin = Math.abs(MathUtil.angleModulus(normalizedDesired - normalizedMin));
        double distanceToMax = Math.abs(MathUtil.angleModulus(normalizedDesired - normalizedMax));
        return distanceToMin <= distanceToMax ? normalizedMin : normalizedMax;
    }

    private Translation2d getDefaultAimTarget() {
        Pose2d robotPose = drivetrain.getRobotPosition();
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        boolean onOurHalf = alliance == Alliance.Blue
                ? robotPose.getX() <= Constants.FIELD_LENGTH / 2.0
                : robotPose.getX() >= Constants.FIELD_LENGTH / 2.0;

        if (onOurHalf) {
            return shooterAimer.getHubPositionFromAlliance();
        }

    boolean isLeftSide = robotPose.getY() <= Constants.FIELD_WIDTH / 2.0;
        if (alliance == Alliance.Blue) {
            return isLeftSide ? Constants.blueLeftShuttlePosition : Constants.blueRightShuttlePosition;
        }
        return isLeftSide ? Constants.redLeftShuttlePosition : Constants.redRightShuttlePosition;
    }

    private double radiansToMotorRotations(double turretAngleRadians) {
        double turretRotations = turretAngleRadians / (2.0 * Math.PI);
        return turretRotations * Constants.turretGearRatio;
    }

    private void configureMotor() {
        TalonFXConfiguration talonMotorConfig = new TalonFXConfiguration();
        talonMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        talonMotorConfig.Slot0 = slot0Configs;
        talonMotorConfig.Voltage.PeakForwardVoltage = Constants.falconMaxVoltage;
        talonMotorConfig.Voltage.PeakReverseVoltage = -Constants.falconMaxVoltage;
        talonMotorConfig.Voltage.SupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;

        talonMotorConfig.CurrentLimits.StatorCurrentLimit = HardwareConstants.ctreStatorCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimit = HardwareConstants.ctreSupplyCurrentMaximumAmps;
        talonMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        talonMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talonMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talonMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = radiansToMotorRotations(maxTurretAngleRadians);
        talonMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = radiansToMotorRotations(minTurretAngleRadians);

        StatusCode response = turretMotor.getConfigurator().apply(talonMotorConfig);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + turretMotor.getDeviceID() + " failed config with error "
                            + response.toString());
        }
    }
}
