// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ManualInputInterfaces.java
// Intent: Forms a class with trajcetories for testing swerve drives.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A class with trajcetories for testing swerve drives.
 */
public class TestTrajectories {
  public PathPlannerPath traverseSimpleForward;
  public PathPlannerPath traverseZigZag;
  public PathPlannerPath traverseSimpleLeft;
  public PathPlannerPath traverseForwardArc;
  public PathPlannerPath traverseBackwardArc;
  public PathPlannerPath oneMeter;
  public PathPlannerPath twoMeter;
  public PathPlannerPath threeMeter;

  private Pose2d traverseBackwardArcStartPosition = new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0.0));

  // TODO These velocities and acccelerations were copied from Ted. May need to be
  // changed for new robot.
  private double maxVelocityMPS = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
  private double maxAccelerationPMSSq = 1.25; // 6.0 max
  private double maxAngularVelocityRadPerSecond = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
  private double maxAngularAccelerationRadPerSecondSq = 8.0; // 12.0 max

  /**
   * constructs trajcetories for testing swerve drives.
   */
  public TestTrajectories() {
    traverseZigZag = buildZigZag();
    traverseSimpleLeft = buildTraverseSimpleLeft();
    traverseForwardArc = buildTraverseForwardArc();
    traverseBackwardArc = buildTraverseBackwardArc();

    oneMeter = buildOneMeter();
    twoMeter = buildTwoMeter();
    threeMeter = buildThreeMeter();
  }

  private PathPlannerPath buildOneMeter() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
        new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0)));

    // Create the path using the waypoints created above
    PathPlannerPath p = new PathPlannerPath(
        waypoints,
        getPathConstraints(),
        null, // do not specify ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    p.preventFlipping = true;

    return p;
  }

  private PathPlannerPath buildTwoMeter() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
        new Pose2d(2.0, 0.0, Rotation2d.fromDegrees(0)));

    // Create the path using the waypoints created above
    PathPlannerPath p = new PathPlannerPath(
        waypoints,
        getPathConstraints(),
        null, // do not specify ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    p.preventFlipping = true;

    return p;
  }

  private PathPlannerPath buildThreeMeter() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, 0.0, Rotation2d.fromDegrees(0)));

    // Create the path using the waypoints created above
    PathPlannerPath p = new PathPlannerPath(
        waypoints,
        getPathConstraints(),
        null, // do not specify ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    p.preventFlipping = true;

    return p;
  }

  private PathPlannerPath buildZigZag() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        // maybe need to add rotations to get the right direction of travel?
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
        new Pose2d(1.5, 0.5, Rotation2d.fromDegrees(0)),
        new Pose2d(3.0, -0.5, Rotation2d.fromDegrees(0)));

    // Create the path using the waypoints created above
    PathPlannerPath p = new PathPlannerPath(
        waypoints,
        getPathConstraints(),
        null, // do not specify ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    p.preventFlipping = true;

    return p;
  }

  private PathPlannerPath buildTraverseSimpleLeft() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        new Pose2d(0.0, 1.0, Rotation2d.fromDegrees(0.0)));

    // Create the path using the waypoints created above
    PathPlannerPath p = new PathPlannerPath(
        waypoints,
        getPathConstraints(),
        null, // do not specify ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    p.preventFlipping = true;

    return p;
  }

  private PathPlannerPath buildTraverseForwardArc() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        new Pose2d(0.5, 0.25, Rotation2d.fromDegrees(0.0)),
        new Pose2d(1.0, 0.50, Rotation2d.fromDegrees(0)),
        new Pose2d(1.5, 0.25, Rotation2d.fromDegrees(0)),
        this.traverseBackwardArcStartPosition);

    // Create the path using the waypoints created above
    PathPlannerPath p = new PathPlannerPath(
        waypoints,
        getPathConstraints(),
        null, // do not specify ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    p.preventFlipping = true;

    return p;
  }

  private PathPlannerPath buildTraverseBackwardArc() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not
    // use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        this.traverseBackwardArcStartPosition,
        new Pose2d(1.5, 0.25, Rotation2d.fromDegrees(0)),
        new Pose2d(1.0, 0.50, Rotation2d.fromDegrees(0)),
        new Pose2d(0.5, 0.25, Rotation2d.fromDegrees(0.0)),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));

    // Create the path using the waypoints created above
    PathPlannerPath p = new PathPlannerPath(
        waypoints,
        getPathConstraints(),
        null, // do not specify ideal starting state
        new GoalEndState(0.0, Rotation2d.fromDegrees(0)) // Goal end state. You can set a holonomic rotation here.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    p.preventFlipping = true;

    return p;
  }

  /**
   * A method to return a new path constraint with the default values
   * 
   * @return PathConstraints
   */
  private PathConstraints getPathConstraints() {
    return new PathConstraints(
        maxVelocityMPS,
        maxAccelerationPMSSq,
        maxAngularVelocityRadPerSecond,
        maxAngularAccelerationRadPerSecondSq);
  }
}
