// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: CameraSubsystem.java
// Intent: Forms the prelminary code for the camera subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.common.VisionMeasurement;
import frc.robot.control.Constants;
import frc.robot.common.DistanceMeasurement;

/**
 * A class to encapsulate the camera subsystem
 */
public class CameraSubsystem extends SubsystemBase {
  private final double milisecondsInSeconds = 1000.0;
  private final double microsecondsInSeconds = 1000000.0;
  private final int TagDoubleArraySize = 7;
  private final int BotposeDoubleArraySize = 8;
  private final int latencyIndex = 6;
  private final int tagCountIndex = 7;
  private final int tagSpaceXIndex = 0;
  private final int tagSpaceYIndex = 2;
  private final int fieldSpaceXIndex = 0;
  private final int fieldSpaceYIndex = 1;
  private final int botRotationIndex = 5;
  private final int noTagInSightId = -1;
  // we use this for teleop vision udpates with an origin in the bottom left blue
  // side
  private String botPoseSource = "botpose_wpiblue";
  // we use this for disabled vision seeding with an origin in the bottom left
  // blue side
  private String botPoseOrbSource = "botpose_orb_wpiblue";
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /**
   * a constructor for the camera subsystem class
   */
  public CameraSubsystem() {
  }

  /**
   * a method that returns a vision measurement.
   * pose portion of the vision measurement is null if there is no valid
   * measurement.
   * 
   * @return a vision measurement of the bot pose in field space
   */
  public VisionMeasurement getVisionBotPose() {
    double tagId = this.table.getEntry("tid").getDouble(noTagInSightId);
    NetworkTableEntry botposeEntry = this.table.getEntry(botPoseSource);
    VisionMeasurement visionMeasurement = new VisionMeasurement(null, 0.0);

    if (botposeEntry.exists() && tagId != noTagInSightId) {
      double[] botpose = botposeEntry.getDoubleArray(new double[this.BotposeDoubleArraySize]);
      Double timestamp = (botposeEntry.getLastChange() / this.microsecondsInSeconds)
          - (botpose[this.latencyIndex] / this.milisecondsInSeconds);
      Translation2d botTranslation = new Translation2d(botpose[this.fieldSpaceXIndex], botpose[this.fieldSpaceYIndex]);
      Rotation2d botYaw = Rotation2d.fromDegrees(botpose[this.botRotationIndex]).plus(Rotation2d.fromDegrees(-90));
      Pose2d realRobotPosition = new Pose2d(botTranslation, botYaw);
      visionMeasurement = new VisionMeasurement(realRobotPosition, Utils.fpgaToCurrentTime(timestamp));
    }
    return visionMeasurement;
  }

  public VisionMeasurement getVisionBotPoseOrb() {
    double tagId = this.table.getEntry("tid").getDouble(noTagInSightId);
    NetworkTableEntry botposeEntry = this.table.getEntry(botPoseOrbSource);
    VisionMeasurement visionMeasurement = new VisionMeasurement(null, 0.0);

    if (botposeEntry.exists() && tagId != noTagInSightId) {
      double[] botpose = botposeEntry.getDoubleArray(new double[this.BotposeDoubleArraySize]);
      Double timestamp = (botposeEntry.getLastChange() / this.microsecondsInSeconds)
          - (botpose[this.latencyIndex] / this.milisecondsInSeconds);
      Translation2d botTranslation = new Translation2d(botpose[this.fieldSpaceXIndex], botpose[this.fieldSpaceYIndex]);
      Rotation2d botYaw = Rotation2d.fromDegrees(botpose[this.botRotationIndex]).plus(Rotation2d.fromDegrees(-90));
      Pose2d realRobotPosition = new Pose2d(botTranslation, botYaw);
      visionMeasurement = new VisionMeasurement(realRobotPosition, Utils.fpgaToCurrentTime(timestamp));
    }
    return visionMeasurement;
  }

  /**
   * a method that gets botPoseSource
   * 
   * @return which limelight datatable we are using
   */
  public String getBotPoseSource() {
    return botPoseSource;
  }

  /**
   * a method that sets which limelight data table we should be using
   * based on the alliance spit out from driver station
   * https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api#apriltag-and-3d-data
   */
  /*
   * public void setBotPoseSource(){
   * var alliance = DriverStation.getAlliance();
   * if(alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red){
   * botPoseSource = "botpose_wpired";
   * }
   * else if(alliance.isPresent() && alliance.get() ==
   * DriverStation.Alliance.Blue){
   * botPoseSource = "botpose_wpiblue";
   * }
   * else{
   * botPoseSource = "botpose";
   * }
   * }
   */

  /**
   * a method that returns the tag id of the current viewed tag
   * 
   * @return double of the current in view tag id
   */
  public double getTagId() {
    return table.getEntry("tid").getDouble(0);
  }

  /**
   * a method that returns the robots distance from one of the given tags
   * 
   * @param blueTId
   * @param redTId
   * @return a ditance measuremtn of the distance, with false if the tag is
   *         invalid
   */
  public DistanceMeasurement getDistanceFromTag(double blueTId, double redTId) {
    DistanceMeasurement measurement = new DistanceMeasurement(false, 0.0);
    Pose2d botPoseIntargetSpace = getVisionBotPoseInTargetSpace();
    if ((getTagId() == blueTId || getTagId() == redTId) && botPoseIntargetSpace != null) {
      measurement.setIsValid(true);

      double xDistance = botPoseIntargetSpace.getTranslation().getX();
      double yDistance = botPoseIntargetSpace.getTranslation().getY();
      double totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
      measurement.setDistanceMeteres(totalDistance);
    }
    return measurement;
  }

  /**
   * a method that returns a pose2d of the robot pose in target space.
   * pose portion of the vision measurement is null if there is no valid
   * measurement.
   */
  public Pose2d getVisionBotPoseInTargetSpace() {
    double tagId = table.getEntry("tid").getDouble(noTagInSightId);
    double[] botpose = this.table.getEntry("botpose_targetspace").getDoubleArray(new double[this.TagDoubleArraySize]);
    Translation2d botTranslation = new Translation2d(botpose[this.tagSpaceXIndex], botpose[this.tagSpaceYIndex]);
    Rotation2d botYaw = Rotation2d.fromDegrees(botpose[this.botRotationIndex]);
    Pose2d realRobotPosition = new Pose2d(botTranslation, botYaw);

    if (tagId == noTagInSightId) {
      return null;
    } else {
      return realRobotPosition;
    }
  }

  /**
   * Translates the given Limelight pose to the WPI Blue coordinate system.
   *
   * @param LLPose The pose obtained from the Limelight camera.
   * @return A new Pose2d object representing the translated pose in the WPI Blue
   *         coordinate system.
   */
  public static Pose2d translateLimelightPoseToWPIBlue(Pose2d LLPose) {
    return new Pose2d(
        LLPose.getTranslation()
            .plus(new Translation2d(Constants.limelightToWPIBlueXOffest, Constants.limelightToWPIBlueYOffset)),
        LLPose.getRotation());
  }

  /**
   * A method to run during periodic for the camera subsystem
   * it displays camera-based position to the shuffleboard
   */
  @Override
  public void periodic() {
  }
}
