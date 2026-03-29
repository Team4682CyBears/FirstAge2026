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
    private Command Bump;
    private Command StraightDepo;

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
            autonomousPathChooser.addOption("Bump", AutonomousPath.BUMP);
            autonomousPathChooser.addOption("StraightDepo", AutonomousPath.STRAIGHTDEPO);
            SmartDashboard.putData(autonomousPathChooser);

            this.DoNothing = getDoNothing();
            this.JustShoot = getJustShoot(subsystems);
            this.BotWingPoo = getBotWingPoo();
            this.TopWingPoo = getTopWingPoo();
            this.HubOutpostDepo = getHubOutpostDepo();
            this.Bump = getBumpAuto();
            this.StraightDepo = getStraightDepo();
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
            case BUMP:
                return this.Bump;
            case STRAIGHTDEPO:
                return this.StraightDepo;
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
        if (InstalledHardware.shooterInstalled && InstalledHardware.hoodMotorInstalled
                && InstalledHardware.hoodEncoderInstalled) {
            Command aim = new ShooterManualCommand(
                    subsystems).withTimeout(15.0);
            Command shoot = new SequentialCommandGroup(
                    new WaitCommand(1),
                    new KickerSpindexerCommand(
                            subsystems.getKickerSubsystem(),
                            subsystems.getSpindexerSpinnerSubsystem())
                            .withTimeout(14.0));
            return new ParallelCommandGroup(aim, shoot);
        } else {
            return new InstantCommand();
        }
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

    private Command getBumpAuto(){
        return AutoBuilder.buildAuto("Bump");
    }

    private Command getStraightDepo(){
        return AutoBuilder.buildAuto("StraightDepo");
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
        BUMP,
        STRAIGHTDEPO
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
                subsystems.getDriveTrainSubsystem()::getChassisSpeedsRobotCentric, // ChassisSpeeds supplier. MUST BE
                                                                                   // ROBOT RELATIVE
                (speeds, feedforwards) -> subsystems.getDriveTrainSubsystem().driveRobotCentric(speeds),
                Constants.pathFollower,
                subsystems.getDriveTrainSubsystem().getPathPlannerConfig(),
                () -> getShouldMirrorPath(),
                subsystems.getDriveTrainSubsystem());

        // Register named commands
        if (subsystems.isDriveTrainSubsystemAvailable()
                && subsystems.isDriveTrainPowerSubsystemAvailable()
                && subsystems.isHoodSubsystemAvailable()
                && subsystems.isShooterSubsystemAvailable()
                && subsystems.isTurretSubsystemAvailable()) {

            NamedCommands.registerCommand(
                    "AutoAimOn",
                    new AutoAimCommand(
                            subsystems,
                            subsystems.getShooterAimer()).withTimeout(5.0));
            
            NamedCommands.registerCommand(
                    "AutoAimOnAndOn",
                    new AutoAimCommand(
                            subsystems,
                            subsystems.getShooterAimer()).withTimeout(15.0));
        }

        if (subsystems.isSpinnerSpindexerSubsystemAvaible()
                && subsystems.isKickerSubsystemAvailable() 
                && subsystems.isIntakeRollerSubsystemAvailable()
                && subsystems.isIntakeWristSubsystemAvailable()) {
            NamedCommands.registerCommand(
                    "SpindexerKickerOn",
                    new KickerSpindexerAgitateCommand(
                            subsystems.getKickerSubsystem(),
                            subsystems.getSpindexerSpinnerSubsystem(),
                            subsystems.getIntakeWristSubsystem(),
                            subsystems.getIntakeRollerSubsystem()).withTimeout(4.8));
        }
        if (subsystems.isSpinnerSpindexerSubsystemAvaible()
            && subsystems.isKickerSubsystemAvailable()){
            NamedCommands.registerCommand(
                    "SpindexerKickerOnAndOn",
                    new KickerSpindexerCommand(
                            subsystems.getKickerSubsystem(),
                            subsystems.getSpindexerSpinnerSubsystem()).withTimeout(14.7));
        }

        if (subsystems.isIntakeWristSubsystemAvailable()
                && subsystems.isIntakeRollerSubsystemAvailable()) {
            NamedCommands.registerCommand(
                    "IntakeToggle",
                    new ToggleIntakeDeployCommand(
                            subsystems.getIntakeWristSubsystem(),
                            subsystems.getIntakeRollerSubsystem()));
        }

        if (subsystems.isShooterSubsystemAvailable()) {
            NamedCommands.registerCommand("RevShooter",
                    new ShootCommand(subsystems.getShooterSubsystem(), () -> 3000.0));
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