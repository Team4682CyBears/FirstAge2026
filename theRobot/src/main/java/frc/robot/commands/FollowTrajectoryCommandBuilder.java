// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: FollowTrajectoryCommandBuilder.java
// Intent: A builder that returns a command to follow a PathPlanner trajectory
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.Optional;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.control.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Forms a class to buid trajectory-following commands
 */
public class FollowTrajectoryCommandBuilder {

    /**
     * A method to build a follow trajectory command
     * 
     * @param traj        - path planner trajectory
     * @param drivetrain  - drivetrain subsystem
     * @param isFirstPath - true if this is the first path in the sequence (resets
     *                    the odometry to traj starting point)
     * @return - a command to follow the trajectory
     */
    public static Command build(PathPlannerPath traj, DrivetrainSubsystem drivetrain, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        setRobotPosition(drivetrain, traj.getStartingHolonomicPose());
                    }
                }),
                new FollowPathCommand(
                        traj,
                        drivetrain::getRobotPosition, // Pose supplier
                        drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        (speeds, feedforwards) -> drivetrain.driveRobotCentric(speeds), // Method that will drive the
                                                                                        // robot given ROBOT RELATIVE
                                                                                        // ChassisSpeeds.
                        // We do not currently use the module feedforwards
                        Constants.pathFollower,
                        drivetrain.getPathPlannerConfig(), // The robot configuration
                        () -> mirrorPathForRedAliance(),
                        (Subsystem) drivetrain));
    }

    /**
     * A method to build a follow trajectory command
     * 
     * @param traj       - path planner trajectory
     * @param drivetrain - drivetrain subsystem
     * @return - a command to follow the trajectory
     */
    public static Command build(PathPlannerPath traj, DrivetrainSubsystem drivetrain) {
        return build(traj, drivetrain, false);
    }

    /**
     * A method that returns true when we are on the red alliance
     * Used for paths that should be mirrored when we are on red alliance
     */
    public static boolean mirrorPathForRedAliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Red;
        }

        return false;
    }

    /**
     * A method that always returns false
     * Used for paths that should never be mirrored
     */
    public static boolean neverMirrorPath() {
        // when using paths generated from april tag coords always turn mirroring off
        return false;
    }

    /**
     * A method that sets robot position to pose if the pose is isPresent
     * Otherwise does not set pose.
     * Handles optional Pose2D in PathPlanner starting pose
     * 
     * @param drivetrain   - the drivetrainSubsystem
     * @param optionalPose - the Optional <Pose2d> returned by PathPlanner
     */
    private static void setRobotPosition(DrivetrainSubsystem drivetrain, Optional<Pose2d> optionalPose) {
        if (optionalPose.isPresent()) {
            drivetrain.setRobotPosition(optionalPose.get());
        }
    }
}