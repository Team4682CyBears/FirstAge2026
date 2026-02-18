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
    private Command OutpostMidBotClimb;
    private Command OutpostMidTopClimb;
    private Command OutpostDepotTopClimb;

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
            autonomousPathChooser.addOption("Drop To Mid", AutonomousPath.DROPTOMID);
            SmartDashboard.putData(autonomousPathChooser);

            this.DoNothing = getDoNothing();
            this.OutpostMidBotClimb = getOutpostMidBotClimb();
            this.OutpostMidTopClimb = getOutpostMidTopClimb();
            this.OutpostDepotTopClimb = getOutpostDepotTopClimb();
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
            case DROPTOMID:
                return this.OutpostMidBotClimb;
            case OUTPOSTMIDTOPCLIMB:
                return this.OutpostMidTopClimb;
            case OUTPOSTDEPOTTOPCLIMB:
                return this.OutpostDepotTopClimb;
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

    private Command getOutpostMidBotClimb() {
        return AutoBuilder.buildAuto("OutpostMidBotClimb");
    }

    private Command getOutpostMidTopClimb() {
        return AutoBuilder.buildAuto("OutpostMidTopClimb");
    }

    private Command getOutpostDepotTopClimb() {
        return AutoBuilder.buildAuto("OutpostDepotTopClimb");
    }

    private Command getDoNothing() {
        return new InstantCommand();
    }

    private enum AutonomousPath {
        OUTPOSTMIDBOTCLIMB,
        OUTPOSTMIDTOPCLIMB,
        OUTPOSTDEPOTTOPCLIMB,
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
                (speeds, feedforwards) -> subsystems.getDriveTrainSubsystem().driveRobotCentric(speeds), // Method that
                                                                                                         // will drive
                // the robot given ROBOT
                // RELATIVE
                // ChassisSpeeds
                Constants.pathFollower,
                subsystems.getDriveTrainSubsystem().getPathPlannerConfig(),
                () -> getShouldMirrorPath(),
                subsystems.getDriveTrainSubsystem());

        // Register named commands
        if (subsystems.isDriveTrainPowerSubsystemAvailable()) {
            // TODO add named commands here. 
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