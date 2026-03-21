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

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.common.VisionMeasurement;
import frc.robot.control.CameraMode;
import frc.robot.control.Constants;
import frc.robot.generated.LimelightHelpers;

/**
 * A class to encapsulate the camera subsystem
 */
public class CameraSubsystem extends SubsystemBase {

  private static final int MIN_FIDUCIALS_FOR_VISION = 1;

  private final String leftLimelightName = Constants.limelightLeftName;
  private final String rightLimelightName = Constants.limelightRightName;

  private final ArrayList<Double> recentVisionYaws = new ArrayList<Double>();
  private final int recentVisionYawsMaxSize = 15;
  private int lastFiducialCount = 0;
  private double lastMaxFiducialAmbiguity = 0.0;
  private double lastHeartbeatLeft = -1.0;
  private double lastHeartbeatRight = -1.0;
  private String lastPreferredLimelight = null;

  private CameraMode cameraMode = CameraMode.TRACKING;

  private static class PoseSelection {
    private final LimelightHelpers.PoseEstimate poseEstimate;
    private final String preferredLimelight;
    private final int leftCount;
    private final int rightCount;
    private final double leftAmbiguity;
    private final double rightAmbiguity;

    private PoseSelection(
        LimelightHelpers.PoseEstimate poseEstimate,
        String preferredLimelight,
        int leftCount,
        int rightCount,
        double leftAmbiguity,
        double rightAmbiguity) {
      this.poseEstimate = poseEstimate;
      this.preferredLimelight = preferredLimelight;
      this.leftCount = leftCount;
      this.rightCount = rightCount;
      this.leftAmbiguity = leftAmbiguity;
      this.rightAmbiguity = rightAmbiguity;
    }

    private boolean hasTargets() {
      return leftCount > 0 || rightCount > 0;
    }
  }

  /**
   * Constructs the camera subsystem.
   */
  public CameraSubsystem() {
  }

  /**
   * Clears any cached vision yaw history used for seeding.
   */
  public void clearRecentVisionYaws() {
    recentVisionYaws.clear();
  }

  /**
   * Returns the last observed fiducial count from the best camera pair.
   *
   * @return last fiducial count observed
   */
  public int getLastFiducialCount() {
    return lastFiducialCount;
  }

  /**
   * Returns the last observed max fiducial ambiguity used for quality checks.
   *
   * @return last max fiducial ambiguity
   */
  public double getLastMaxFiducialAmbiguity() {
    return lastMaxFiducialAmbiguity;
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
    boolean leftNewFrame = updateHeartbeat(leftLimelightName);
    boolean rightNewFrame = updateHeartbeat(rightLimelightName);
    
    if (!leftNewFrame && !rightNewFrame) {
      return null;
    }

    LimelightHelpers.PoseEstimate leftEstimate = null;
    LimelightHelpers.PoseEstimate rightEstimate = null;

    if (cameraMode == CameraMode.SEEDING) {
      leftEstimate = leftNewFrame ? getPoseEstimateMT1(leftLimelightName) : null;
      rightEstimate = rightNewFrame ? getPoseEstimateMT1(rightLimelightName) : null;
    } else {
      leftEstimate = leftNewFrame ? getPoseEstimateMT2(leftLimelightName) : null;
      rightEstimate = rightNewFrame ? getPoseEstimateMT2(rightLimelightName) : null;
    }

    PoseSelection selection = selectBestPoseEstimateAndPreferred(leftEstimate, rightEstimate);
    updateFiducialDiagnostics(
        selection.leftCount,
        selection.leftAmbiguity,
        selection.rightCount,
        selection.rightAmbiguity);
    updatePreferredLimelightLEDs(selection);

    if (cameraMode == CameraMode.SEEDING) {
      // use MT1 yaw from the single best frame to avoid mixing disagreements
      if (selection.poseEstimate != null) {
        updateRecentVisionYaws(toVisionMeasurement(selection.poseEstimate));
      }
      
      // seed measurement uses MT2 position and recent Yaws (averaged from MT1)
      VisionMeasurement seedMeasurement = getSeedPoseFromVision();
      if (seedMeasurement != null && seedMeasurement.getRobotPosition() != null) {
        updateCameraIMU(seedMeasurement.getRobotPosition().getRotation());
      }
      return seedMeasurement;
    } 
    else { // TRACKING MODE
      updateCameraIMU(gyroRotation);
      return toVisionMeasurement(selection.poseEstimate);
    }
  }

  /**
   * Returns the current camera mode.
   *
   * @return current camera mode
   */
  public CameraMode getMode() {
    return cameraMode;
  }

  /**
   * Returns the latest MegaTag1 pose estimate using the best camera frame.
   *
   * @return MT1 VisionMeasurement
   */
  public VisionMeasurement getVisionBotPoseMT1() {
    LimelightHelpers.PoseEstimate left = getPoseEstimateMT1(leftLimelightName);
    LimelightHelpers.PoseEstimate right = getPoseEstimateMT1(rightLimelightName);
    PoseSelection selection = selectBestPoseEstimateAndPreferred(left, right);
    return toVisionMeasurement(selection.poseEstimate);
  }

  /**
   * Returns the latest MegaTag2 pose estimate using the best camera frame.
   *
   * @return MT2 VisionMeasurement
   */
  public VisionMeasurement getVisionBotPoseMT2() {
    LimelightHelpers.PoseEstimate left = getPoseEstimateMT2(leftLimelightName);
    LimelightHelpers.PoseEstimate right = getPoseEstimateMT2(rightLimelightName);
    PoseSelection selection = selectBestPoseEstimateAndPreferred(left, right);
    return toVisionMeasurement(selection.poseEstimate);
  }

  /**
   * Returns whether the most recent vision data is considered reliable.
   *
   * @return true when fiducial count and ambiguity meet thresholds
   */
  public boolean isVisionQualityGood() {
    return lastFiducialCount >= MIN_FIDUCIALS_FOR_VISION
        && lastMaxFiducialAmbiguity <= Constants.TAG_AMBIGUITY_THRESHOLD;
  }

  /**
   * Periodic update hook for the subsystem.
   */
  @Override
  public void periodic() {
  }

  /**
   * Updates the Limelight mode and IMU assist settings.
   *
   * @param newMode mode to switch into
   */
  public void setMode(CameraMode newMode) {
    if (cameraMode == newMode) return;
    
    if (newMode == CameraMode.SEEDING) {
      LimelightHelpers.SetIMUMode(leftLimelightName, 1);
      LimelightHelpers.SetIMUMode(rightLimelightName, 1);
      clearRecentVisionYaws();
    } else if (newMode == CameraMode.TRACKING) {
      LimelightHelpers.SetIMUMode(leftLimelightName, 4);
      LimelightHelpers.SetIMUMode(rightLimelightName, 4);
      LimelightHelpers.SetIMUAssistAlpha(leftLimelightName, Constants.IMUassistAlpha);
      LimelightHelpers.SetIMUAssistAlpha(rightLimelightName, Constants.IMUassistAlpha);
    } 
    this.cameraMode = newMode;
  }

  /**
   * Applies the WPI Blue field translation offset to a Limelight pose.
   *
   * @param LLPose limelight pose to translate
   * @return translated pose in WPI Blue coordinates
   */
  public static Pose2d translateLimelightPoseToWPIBlue(Pose2d LLPose) {
    return new Pose2d(
        LLPose.getTranslation()
            .plus(new Translation2d(Constants.limelightToWPIBlueXOffest, Constants.limelightToWPIBlueYOffset)),
        LLPose.getRotation());
  }

  private double getBestAmbiguity(int leftCount, double leftAmb, int rightCount, double rightAmb) {
    if (leftCount == 0 && rightCount == 0) return 0.0;
    if (leftCount == 0) return rightAmb;
    if (rightCount == 0) return leftAmb;
    return Math.min(leftAmb, rightAmb);
  }

  private double getMaxRawFiducialAmbiguity(LimelightHelpers.RawFiducial[] raw) {
    if (raw == null || raw.length == 0) {
      return 0.0;
    }
    double max = 0.0;
    for (LimelightHelpers.RawFiducial r : raw) {
      if (r != null && r.ambiguity > max) {
        max = r.ambiguity;
      }
    }
    return max;
  }

  private Double getMedianOfList(ArrayList<Double> list) {
    ArrayList<Double> modifiedList = new ArrayList<Double>(list);
    Collections.sort(modifiedList);
    return modifiedList.get((int) (modifiedList.size() / 2));
  }

  private LimelightHelpers.PoseEstimate getPoseEstimateMT1(String limelightName) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
  }

  private LimelightHelpers.PoseEstimate getPoseEstimateMT2(String limelightName) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
  }

  private VisionMeasurement getSeedPoseFromVision() {
    if (recentVisionYaws.isEmpty()) return null;

    double medianYaw = getMedianOfList(recentVisionYaws);
    VisionMeasurement MT2 = getVisionBotPoseMT2();
    Pose2d MT2pose = MT2.getRobotPosition();
    double MT2timestamp = MT2.getTimestamp();

    if (MT2pose == null) return null;

    return new VisionMeasurement(
        new Pose2d(MT2pose.getTranslation(), Rotation2d.fromDegrees(medianYaw)),
        MT2timestamp);
  }

  private PoseSelection selectBestPoseEstimateAndPreferred(
      LimelightHelpers.PoseEstimate left,
      LimelightHelpers.PoseEstimate right) {
    int leftCount = left != null ? left.tagCount : 0;
    int rightCount = right != null ? right.tagCount : 0;
    double leftAmbiguity = left != null ? getMaxRawFiducialAmbiguity(left.rawFiducials) : 0.0;
    double rightAmbiguity = right != null ? getMaxRawFiducialAmbiguity(right.rawFiducials) : 0.0;

    if (left == null && right == null) {
      return new PoseSelection(null, null, leftCount, rightCount, leftAmbiguity, rightAmbiguity);
    }

    boolean preferLeft;
    if (left == null) {
      preferLeft = false;
    } else if (right == null) {
      preferLeft = true;
    } else if (leftCount != rightCount) {
      preferLeft = leftCount > rightCount;
    } else if (leftCount == 0 && rightCount == 0) {
      preferLeft = true;
    } else if (leftAmbiguity != rightAmbiguity) {
      preferLeft = leftAmbiguity < rightAmbiguity;
    } else {
      preferLeft = left.timestampSeconds >= right.timestampSeconds;
    }

    LimelightHelpers.PoseEstimate bestPose = preferLeft ? left : right;
    String preferred = preferLeft ? leftLimelightName : rightLimelightName;
    return new PoseSelection(bestPose, preferred, leftCount, rightCount, leftAmbiguity, rightAmbiguity);
  }

  private VisionMeasurement toVisionMeasurement(LimelightHelpers.PoseEstimate poseEstimate) {
    if (poseEstimate != null && poseEstimate.pose != null) {
      double fpgaTime = Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds);
      return new VisionMeasurement(poseEstimate.pose, fpgaTime);
    }
    return new VisionMeasurement(null, 0.0);
  }

  private void updateCameraIMU(Rotation2d yaw) {
    LimelightHelpers.SetRobotOrientation(leftLimelightName, yaw.getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(rightLimelightName, yaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  private void updateFiducialDiagnostics(int leftCount, double leftAmb, int rightCount, double rightAmb) {
    lastFiducialCount = Math.max(leftCount, rightCount);
    lastMaxFiducialAmbiguity = getBestAmbiguity(leftCount, leftAmb, rightCount, rightAmb);
  }

  private boolean updateHeartbeat(String limelightName) {
    double heartbeat = LimelightHelpers.getHeartbeat(limelightName);
    if (limelightName.equals(leftLimelightName)) {
      boolean hasNewFrame = heartbeat != lastHeartbeatLeft;
      lastHeartbeatLeft = heartbeat;
      return hasNewFrame;
    }
    boolean hasNewFrame = heartbeat != lastHeartbeatRight;
    lastHeartbeatRight = heartbeat;
    return hasNewFrame;
  }

  private void updatePreferredLimelightLEDs(PoseSelection selection) {
    String preferred = selection.preferredLimelight;
    if (preferred == null) {
      lastPreferredLimelight = null;
      LimelightHelpers.setLEDMode_PipelineControl(leftLimelightName);
      LimelightHelpers.setLEDMode_PipelineControl(rightLimelightName);
      return;
    }
    if (preferred.equals(lastPreferredLimelight)) {
      LimelightHelpers.setLEDMode_PipelineControl(leftLimelightName);
      LimelightHelpers.setLEDMode_PipelineControl(rightLimelightName);
      return;
    }

    if (!selection.hasTargets()) {
      LimelightHelpers.setLEDMode_PipelineControl(leftLimelightName);
      LimelightHelpers.setLEDMode_PipelineControl(rightLimelightName);
    } else if (preferred.equals(leftLimelightName)) {
      LimelightHelpers.setLEDMode_ForceOn(leftLimelightName);
      LimelightHelpers.setLEDMode_ForceOff(rightLimelightName);
    } else {
      LimelightHelpers.setLEDMode_ForceOff(leftLimelightName);
      LimelightHelpers.setLEDMode_ForceOn(rightLimelightName);
    }

    lastPreferredLimelight = preferred;
  }

  private void updateRecentVisionYaws(VisionMeasurement visionMeasurement) {
    if (visionMeasurement != null && visionMeasurement.getRobotPosition() != null) {
      recentVisionYaws.add(visionMeasurement.getRobotPosition().getRotation().getDegrees());
      while (recentVisionYaws.size() > recentVisionYawsMaxSize) {
        recentVisionYaws.remove(0);
      }
    }
  }
}