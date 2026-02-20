// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: AutonomousChooser.java
// Intent: Allows auto mode routine to be selected from shuffleboard
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.commands.*;

/**
 * a class for choosing different auto modes from shuffleboard
 */
public class AutonomousChooser {
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();
    private Command DoNothing;
    private Command HubOutpostDepoClimb;
    private Command BotWingClimb;
    private Command BotLoopClimb;
    private Command TopWingClimb;
    private Command TopLoopClimb;
    
    /**
     * Constructor for AutonomousChooser
     * 
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems) {
        // TODO add checks for all subsystems the autos rely on besides the drivetrain
        // here
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {
            autonomousPathChooser.setDefaultOption("Do Nothing", AutonomousPath.DONOTHING);
            autonomousPathChooser.addOption("HubOutpostDepoClimb", AutonomousPath.HUBOUTPOSTDEPOCLIMB);
            autonomousPathChooser.addOption("Bot Wing", AutonomousPath.BOTWINGCLIMB);
            autonomousPathChooser.addOption("Bot Loop", AutonomousPath.BOTLOOPCLIMB);
            autonomousPathChooser.addOption("Top Wing", AutonomousPath.TOPWINGCLIMB);
            autonomousPathChooser.addOption("Top Loop", AutonomousPath.TOPLOOPCLIMB);
            SmartDashboard.putData(autonomousPathChooser);

            this.DoNothing = getDoNothing();
            this.HubOutpostDepoClimb = getHubOutpostDepoClimb();
            this.BotWingClimb = getBotWingClimb();
            this.BotLoopClimb = getBotLoopClimb();
            this.TopWingClimb = getTopWingClimb();
            this.TopLoopClimb = getTopLoopClimb();
        } else {
            DataLogManager.log(">>>>> NO auto routine becuase missing subsystems");
        }
    }   

    /**
     * returns the path planner auto to be used in auto period
     * 
     * @return command
     */
    public Command getAutoPath() {
        switch (autonomousPathChooser.getSelected()) {
            case DONOTHING:
                return this.DoNothing;
            case HUBOUTPOSTDEPOCLIMB:
                return this.HubOutpostDepoClimb;
            case BOTWINGCLIMB:
                return this.BotWingClimb;
            case BOTLOOPCLIMB:
                return this.BotLoopClimb;
            case TOPWINGCLIMB:
                return this.TopWingClimb;
            case TOPLOOPCLIMB:
                return this.TopLoopClimb;
        }
        return new InstantCommand();
    }

    /**
     * A method to return the chosen auto command
     * 
     * @return command
     */
    public Command getCommand() {
        return new ParallelCommandGroup(
                getAutoPath());
    }

    private Command getHubOutpostDepoClimb() {
        return AutoBuilder.buildAuto("HubOutpostDepoClimb");
    }
    
    private Command getBotWingClimb() {
        return AutoBuilder.buildAuto("BotWingClimb");
    }

    private Command getBotLoopClimb() {
        return AutoBuilder.buildAuto("BotLoopClimb");
    }

    private Command getTopWingClimb() {
        return AutoBuilder.buildAuto("TopWingClimb");
    }

    private Command getTopLoopClimb() {
        return AutoBuilder.buildAuto("TopLoopClimb");
    }

    private Command getDoNothing() {
        return new InstantCommand();
    }

    private enum AutonomousPath {
        HUBOUTPOSTDEPOCLIMB,
        BOTWINGCLIMB,
        BOTLOOPCLIMB,
        TOPWINGCLIMB,
        TOPLOOPCLIMB,
        DONOTHING,
    }

    /**
     * configures the PIDs and stuff to be used for autonomous driving
     * 
     * @param subsystems
     */
    public static void configureAutoBuilder(SubsystemCollection subsystems) {

        AutoBuilder.configure(
                subsystems.getDriveTrainSubsystem()::getRobotPosition, // Pose supplier
                subsystems.getDriveTrainSubsystem()::setRobotPosition, // Position setter
                subsystems.getDriveTrainSubsystem()::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> subsystems.getDriveTrainSubsystem().driveRobotCentric(speeds), 
                Constants.pathFollower,
                subsystems.getDriveTrainSubsystem().getPathPlannerConfig(),
                () -> getShouldMirrorPath(),
                subsystems.getDriveTrainSubsystem());

        // Register named commands
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {
            NamedCommands.registerCommand("StartAiming", 
                new InstantCommand(() -> {
                    subsystems.getDriveTrainSubsystem().setSwerveYawMode(SwerveYawMode.AUTO);
                    com.pathplanner.lib.controllers.PPHolonomicDriveController.overrideRotationFeedback(
                        () -> subsystems.getDriveTrainSubsystem().getAutoYawVelocityRadiansPerSecond()
                    );
                }));

            NamedCommands.registerCommand("StopAiming", 
                new InstantCommand(() -> {
                    subsystems.getDriveTrainSubsystem().setSwerveYawMode(SwerveYawMode.JOYSTICK); 
                    com.pathplanner.lib.controllers.PPHolonomicDriveController.clearRotationFeedbackOverride();
                }));
        }
    }

    public static boolean getShouldMirrorPath() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }

        return false;
    }
}