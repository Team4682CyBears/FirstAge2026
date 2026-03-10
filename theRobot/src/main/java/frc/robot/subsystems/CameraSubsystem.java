// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: CameraSubsystem.java
// Intent: Forms the prelminary code for the camera subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.common.VisionMeasurement;
import frc.robot.control.CameraMode;
import frc.robot.control.Constants;
import frc.robot.generated.LimelightHelpers;
import frc.robot.common.DistanceMeasurement;

/**
 * A class to encapsulate the camera subsystem
 */
public class CameraSubsystem extends SubsystemBase {

  /**
   * Set the internal botPoseSource (NT entry name) based on DriverStation alliance.
   * This mirrors the commented-out logic previously in RobotContainer.
   */

  private final int noTagInSightId = -1;
  private static final int MIN_FIDUCIALS_FOR_VISION = 1;

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  private final ArrayList<Double> recentVisionYaws = new ArrayList<Double>();
  private final int recentVisionYawsMaxSize = 15;
  private int lastFiducialCount = 0;
  private double lastMaxFiducialAmbiguity = 0.0;
  private double lastHeartbeat = -1.0;

  private CameraMode cameraMode = CameraMode.TRACKING;

  /**
   * a constructor for the camera subsystem class
   */
  public CameraSubsystem() {
  }

  /**
   * a method that returns the MT1 vision measurement.
   * pose portion of the vision measurement is null if there is no valid
   * measurement.
   * 
   * @return a vision measurement of the MT1 bot pose in field space
   */
  public VisionMeasurement getVisionBotPoseMT1() {
    // Use LimelightHelpers to get a PoseEstimate (includes timestamp + latency handling)
    VisionMeasurement visionMeasurement = new VisionMeasurement(null, 0.0);
    // As of 2024, we only use wpiblue
    LimelightHelpers.PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (pe != null && pe.pose != null) {
      // LimelightHelpers timestampSeconds is server-time (seconds since epoch), convert to FPGA time reference
      double fpgaTime = Utils.fpgaToCurrentTime(pe.timestampSeconds);
      visionMeasurement = new VisionMeasurement(pe.pose, fpgaTime);
    }
    return visionMeasurement;
  }

  /**
   * A method that returns the MT2 vision measurement
   * pose position of the vision measurement is null if there is no valid 
   * measurement.
   * 
   * @return a vision measurement of the MT2 bot pose in field space
   */
  public VisionMeasurement getVisionBotPoseMT2() {
    VisionMeasurement visionMeasurement = new VisionMeasurement(null, 0.0);
    // As of 2024, we only use wpiblue
    LimelightHelpers.PoseEstimate pe = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (pe != null && pe.pose != null) {
      double fpgaTime = Utils.fpgaToCurrentTime(pe.timestampSeconds);
      visionMeasurement = new VisionMeasurement(pe.pose, fpgaTime);
    }
    return visionMeasurement;
  }

  /**
   * Returns the latest vision measurement only when a new limelight frame is
   * available. Also refreshes fiducial diagnostics and recent yaw history.
   * Intended to be called by drivetrainSubsystem every periodic
   *
   * @param gyroRotation Current robot field rotation for limelight orientation
   * @return latest VisionMeasurement or null if no new frame was detected
   */
  public VisionMeasurement getLatestVisionMeasurement(Rotation2d gyroRotation) {
    // if there isn't a new measurement, don't update anything
    double heartbeat = LimelightHelpers.getHeartbeat("limelight");
    if (heartbeat == lastHeartbeat) {
      return null;
    }
    lastHeartbeat = heartbeat;
    updateFiducialDiagnostics();

    if (cameraMode == CameraMode.SEEDING) {
      // use MT1 for yaws
      updateRecentVisionYaws(getVisionBotPoseMT1());
      // seed measurement uses MT2 position and recent Yaws (averaged from MT1)
      VisionMeasurement seedMeasurement = getSeedPoseFromVision();
      updateCameraIMU(seedMeasurement.getRobotPosition().getRotation());
      return seedMeasurement;
    } 
    else { // TRACKING MODE
      // use MT2
      VisionMeasurement visionMeasurement = getVisionBotPoseMT2();
      updateRecentVisionYaws(visionMeasurement);
      // use robot yaw for camera IMU
      updateCameraIMU(gyroRotation);
      return visionMeasurement;
    }
  }

  public CameraMode getMode() {
    return cameraMode;
  }

  /**
   * Compute a seeded pose from the recent vision yaw history.
   *
   * @return combined pose with median vision yaw, or null if unavailable
   */
  private VisionMeasurement getSeedPoseFromVision() {
    if (recentVisionYaws.isEmpty()) {
      return null;
    }

    double medianYaw = getMedianOfList(recentVisionYaws);
    VisionMeasurement MT2 = getVisionBotPoseMT2();
    Pose2d MT2pose = MT2.getRobotPosition();
    double MT2timestamp = MT2.getTimestamp();
    if (MT2pose == null) {
      return null;
    }
    return new VisionMeasurement(new Pose2d(MT2pose.getTranslation(), Rotation2d.fromDegrees(medianYaw)),
    MT2timestamp);
  }

  /**
   * a method that returns the tag id of the current viewed tag
   * 
   * @return double of the current in view tag id
   */
  public double getTagId() {
    return table.getEntry("tid").getDouble(0);
  }

  public int getLastFiducialCount() {
    return lastFiducialCount;
  }

  public double getLastMaxFiducialAmbiguity() {
    return lastMaxFiducialAmbiguity;
  }

  public double getLastHeartbeat() {
    return lastHeartbeat;
  }

  /**
   * Returns the maximum ambiguity value from the latest raw fiducials read from
   * the Limelight. If no fiducials are present this returns 0.0.
   *
   * @return maximum ambiguity (double)
   */
  public double getMaxRawFiducialAmbiguity() {
    try {
      LimelightHelpers.RawFiducial[] raw = LimelightHelpers.getRawFiducials("limelight");
      if (raw == null || raw.length == 0) {
        return 0.0;
      }
      double max = 0.0;
      for (LimelightHelpers.RawFiducial r : raw) {
        if (r != null) {
          if (r.ambiguity > max) {
            max = r.ambiguity;
          }
        }
      }
      return max;
    } catch (Exception e) {
      return 0.0;
    }
  }

  public boolean isVisionQualityGood() {
    int fiducialCount = getLastFiducialCount();
    double maxAmbiguity = getLastMaxFiducialAmbiguity();
    return (fiducialCount >= MIN_FIDUCIALS_FOR_VISION
        && maxAmbiguity <= Constants.TAG_AMBIGUITY_THRESHOLD);
  }

  private void updateFiducialDiagnostics() {
    LimelightHelpers.RawFiducial[] rawFiducials = LimelightHelpers.getRawFiducials("limelight");
    lastFiducialCount = rawFiducials == null ? 0 : rawFiducials.length;
    lastMaxFiducialAmbiguity = getMaxRawFiducialAmbiguity();
  }

  private void updateRecentVisionYaws(VisionMeasurement visionMeasurement) {
    if (visionMeasurement != null && visionMeasurement.getRobotPosition() != null) {
      recentVisionYaws.add(visionMeasurement.getRobotPosition().getRotation().getDegrees());
      while (recentVisionYaws.size() > recentVisionYawsMaxSize) {
        recentVisionYaws.remove(0);
      }
    }
  }

  private void updateCameraIMU(Rotation2d yaw){
    LimelightHelpers.SetRobotOrientation("limelight", yaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  public void clearRecentVisionYaws(){
    recentVisionYaws.clear();
  }

  /**
   * Calculates the median value of a list of Double values.
   *
   * @param list the list of Double values to find the median of
   * @return the median value of the list
   */
  private Double getMedianOfList(ArrayList<Double> list) {
    ArrayList<Double> modifiedList = new ArrayList<Double>(list);
    Collections.sort(modifiedList);
    return modifiedList.get((int) (modifiedList.size() / 2));
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
    if (tagId == noTagInSightId) {
      return null;
    }
    double[] botpose = LimelightHelpers.getBotPose_TargetSpace("limelight");
    if (botpose == null || botpose.length == 0) {
      return null;
    }
    return LimelightHelpers.toPose2D(botpose);
  }

  public void setMode(CameraMode newMode) {
    CameraMode previousMode = cameraMode;
    // going to SEEDING from TRACKING
    if (newMode == CameraMode.SEEDING && previousMode == CameraMode.TRACKING) {
      LimelightHelpers.SetIMUMode("limelight", 1); // set limelight IMU to seeding mode
      clearRecentVisionYaws();
    } 
    // going to TRACKING from SEEDING
    else if (newMode == CameraMode.TRACKING && previousMode == CameraMode.SEEDING) {
      LimelightHelpers.SetIMUMode("limelight", 4);
      LimelightHelpers.SetIMUAssistAlpha("limelight", Constants.IMUassistAlpha);
    } 
    this.cameraMode = newMode;
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
    // nothing in here. drivetrainSubsystem should call getLatestVisionMeasurement in its periodic
  }
}
