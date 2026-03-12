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
    private Command JustShoot;
    private Command BotWingPoo;
    private Command TopWingPoo;
    private Command HubOutpostDepo;
    
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
            autonomousPathChooser.addOption("Just Shoot", AutonomousPath.JUSTSHOOT);
            autonomousPathChooser.addOption("Bot Poo", AutonomousPath.BOTWINGPOO);
            autonomousPathChooser.addOption("Top Poo", AutonomousPath.TOPWINGPOO);
            autonomousPathChooser.addOption("Hub Outpost Depo", AutonomousPath.HUBOUTPOSTDEPO);
            SmartDashboard.putData(autonomousPathChooser);

            this.DoNothing = getDoNothing();
            this.JustShoot = getJustShoot(subsystems);
            this.BotWingPoo = getBotWingPoo();
            this.TopWingPoo = getTopWingPoo();
            this.HubOutpostDepo = getHubOutpostDepo();
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
            case JUSTSHOOT:
                return this.JustShoot;
            case BOTWINGPOO:
                return this.BotWingPoo;
            case TOPWINGPOO:
                return this.TopWingPoo;
            case HUBOUTPOSTDEPO:
                return this.HubOutpostDepo;
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

    private Command getJustShoot(SubsystemCollection subsystems) {
        Command aim = new AutoAimMovingCommand(
                subsystems,
                subsystems.getDriveTrainSubsystem().getShooterAimer()).withTimeout(5.0);
        Command shoot = new SequentialCommandGroup(
        new WaitCommand(0.5),
        new KickerSpindexerAgitateCommand(
            subsystems.getKickerSubsystem(),
            subsystems.getSpindexerSpinnerSubsystem(),
            subsystems.getIntakeWristSubsystem())
            .withTimeout(5.0));
        
        return new ParallelCommandGroup(aim, shoot);
    }
    
    private Command getBotWingPoo() {
        return AutoBuilder.buildAuto("BotWingPoo");
    }


    private Command getTopWingPoo() {
        return AutoBuilder.buildAuto("TopWingPoo");
    }

    private Command getHubOutpostDepo() {
        return AutoBuilder.buildAuto("HubOutpostDepo");
    }


    private Command getDoNothing() {
        return new InstantCommand();
    }

    private enum AutonomousPath {
        JUSTSHOOT,
        BOTWINGPOO,
        TOPWINGPOO,
        HUBOUTPOSTDEPO,
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
    if (subsystems.isDriveTrainSubsystemAvailable()
        && subsystems.isDriveTrainPowerSubsystemAvailable()
        && subsystems.isHoodSubsystemAvailable()
        && subsystems.isShooterSubsystemAvailable()) {

        NamedCommands.registerCommand(
            "AutoAimOn",
            new AutoAimMovingCommand(
                subsystems,
                subsystems.getDriveTrainSubsystem().getShooterAimer()).withTimeout(5));
    }

    if (subsystems.isSpinnerSpindexerSubsystemAvaible()
        && subsystems.isKickerSubsystemAvailable()) {
        NamedCommands.registerCommand(
            "SpindexerKickerOn",
            new KickerSpindexerAgitateCommand(
                subsystems.getKickerSubsystem(),
                subsystems.getSpindexerSpinnerSubsystem(),
                subsystems.getIntakeWristSubsystem()).withTimeout(5));
    }

    if (subsystems.isIntakeWristSubsystemAvailable()
        && subsystems.isIntakeRollerSubsystemAvailable()) {
        NamedCommands.registerCommand(
            "IntakeToggle",
            new ToggleIntakeDeployCommand(
                subsystems.getIntakeWristSubsystem(),
                subsystems.getIntakeRollerSubsystem()));
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