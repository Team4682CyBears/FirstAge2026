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

/**
 * A class to encapsulate the camera subsystem
 */
public class CameraSubsystem extends SubsystemBase {

  private final int noTagInSightId = -1;
  private static final int MIN_FIDUCIALS_FOR_VISION = 1;

  private final String leftLimelightName = Constants.limelightLeftName;
  private final String rightLimelightName = Constants.limelightRightName;

  private NetworkTable leftTable = NetworkTableInstance.getDefault().getTable(leftLimelightName);
  private NetworkTable rightTable = NetworkTableInstance.getDefault().getTable(rightLimelightName);

  private final ArrayList<Double> recentVisionYaws = new ArrayList<Double>();
  private final int recentVisionYawsMaxSize = 15;
  private int lastFiducialCount = 0;
  private double lastMaxFiducialAmbiguity = 0.0;
  private double lastHeartbeat = -1.0;
  private double lastHeartbeatLeft = -1.0;
  private double lastHeartbeatRight = -1.0;
  private String lastPreferredLimelight = null;

  private CameraMode cameraMode = CameraMode.TRACKING;

  /**
   * a constructor for the camera subsystem class
   */
  public CameraSubsystem() {
  }

  public VisionMeasurement getVisionBotPoseMT1() {
    LimelightHelpers.PoseEstimate left = getPoseEstimateMT1(leftLimelightName);
    LimelightHelpers.PoseEstimate right = getPoseEstimateMT1(rightLimelightName);
    return toVisionMeasurement(selectBestPoseEstimate(left, right));
  }

  public VisionMeasurement getVisionBotPoseMT2() {
    LimelightHelpers.PoseEstimate left = getPoseEstimateMT2(leftLimelightName);
    LimelightHelpers.PoseEstimate right = getPoseEstimateMT2(rightLimelightName);
    return toVisionMeasurement(selectBestPoseEstimate(left, right));
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
    
    lastHeartbeat = Math.max(lastHeartbeatLeft, lastHeartbeatRight);

    LimelightHelpers.PoseEstimate leftEstimate = null;
    LimelightHelpers.PoseEstimate rightEstimate = null;

    if (cameraMode == CameraMode.SEEDING) {
      leftEstimate = leftNewFrame ? getPoseEstimateMT1(leftLimelightName) : null;
      rightEstimate = rightNewFrame ? getPoseEstimateMT1(rightLimelightName) : null;
    } else {
      leftEstimate = leftNewFrame ? getPoseEstimateMT2(leftLimelightName) : null;
      rightEstimate = rightNewFrame ? getPoseEstimateMT2(rightLimelightName) : null;
    }

    int leftCount = leftEstimate != null ? leftEstimate.tagCount : 0;
    int rightCount = rightEstimate != null ? rightEstimate.tagCount : 0;
    double leftAmb = leftEstimate != null ? getMaxRawFiducialAmbiguity(leftEstimate.rawFiducials) : 0.0;
    double rightAmb = rightEstimate != null ? getMaxRawFiducialAmbiguity(rightEstimate.rawFiducials) : 0.0;

    updateFiducialDiagnostics(leftCount, leftAmb, rightCount, rightAmb);
    updatePreferredLimelightLEDs(leftCount, leftAmb, rightCount, rightAmb);

    if (cameraMode == CameraMode.SEEDING) {
      // use MT1 for yaws from any new frame
      if (leftEstimate != null) updateRecentVisionYaws(toVisionMeasurement(leftEstimate));
      if (rightEstimate != null) updateRecentVisionYaws(toVisionMeasurement(rightEstimate));
      
      // seed measurement uses MT2 position and recent Yaws (averaged from MT1)
      VisionMeasurement seedMeasurement = getSeedPoseFromVision();
      if (seedMeasurement != null && seedMeasurement.getRobotPosition() != null) {
        updateCameraIMU(seedMeasurement.getRobotPosition().getRotation());
      }
      return seedMeasurement;
    } 
    else { // TRACKING MODE
      updateCameraIMU(gyroRotation);
      return toVisionMeasurement(selectBestPoseEstimate(leftEstimate, rightEstimate));
    }
  }

  public CameraMode getMode() {
    return cameraMode;
  }

  private LimelightHelpers.PoseEstimate getPoseEstimateMT1(String limelightName) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
  }

  private LimelightHelpers.PoseEstimate getPoseEstimateMT2(String limelightName) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
  }

  private VisionMeasurement toVisionMeasurement(LimelightHelpers.PoseEstimate poseEstimate) {
    if (poseEstimate != null && poseEstimate.pose != null) {
      double fpgaTime = Utils.fpgaToCurrentTime(poseEstimate.timestampSeconds);
      return new VisionMeasurement(poseEstimate.pose, fpgaTime);
    }
    return new VisionMeasurement(null, 0.0);
  }

  private LimelightHelpers.PoseEstimate selectBestPoseEstimate(
      LimelightHelpers.PoseEstimate left,
      LimelightHelpers.PoseEstimate right) {
    if (left == null) return right;
    if (right == null) return left;
    
    int leftCount = left.tagCount;
    int rightCount = right.tagCount;
    double leftAmbiguity = getMaxRawFiducialAmbiguity(left.rawFiducials);
    double rightAmbiguity = getMaxRawFiducialAmbiguity(right.rawFiducials);

    if (leftCount != rightCount) {
      return leftCount > rightCount ? left : right;
    }
    if (leftCount == 0 && rightCount == 0) {
      return left;
    }
    if (leftAmbiguity != rightAmbiguity) {
      return leftAmbiguity < rightAmbiguity ? left : right;
    }
    return left.timestampSeconds >= right.timestampSeconds ? left : right;
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

  private double getBestAmbiguity(int leftCount, double leftAmb, int rightCount, double rightAmb) {
    if (leftCount == 0 && rightCount == 0) return 0.0;
    if (leftCount == 0) return rightAmb;
    if (rightCount == 0) return leftAmb;
    return Math.min(leftAmb, rightAmb);
  }

  private double getTagId(String limelightName) {
    NetworkTable table = limelightName.equals(leftLimelightName) ? leftTable : rightTable;
    return table.getEntry("tid").getDouble(noTagInSightId);
  }

  private String selectBestLimelightName(int leftCount, double leftAmb, int rightCount, double rightAmb) {
    if (leftCount == 0 && rightCount == 0) {
      return null;
    }
    if (leftCount != rightCount) {
      return leftCount > rightCount ? leftLimelightName : rightLimelightName;
    }
    if (leftAmb != rightAmb) {
      return leftAmb < rightAmb ? leftLimelightName : rightLimelightName;
    }
    return leftLimelightName;
  }

  private Pose2d getVisionBotPoseInTargetSpace(String limelightName) {
    if (limelightName == null) return null;
    double tagId = getTagId(limelightName);
    if (tagId == noTagInSightId) return null;
    
    double[] botpose = LimelightHelpers.getBotPose_TargetSpace(limelightName);
    if (botpose == null || botpose.length == 0) return null;
    
    return LimelightHelpers.toPose2D(botpose);
  }

  private VisionMeasurement getSeedPoseFromVision() {
    if (recentVisionYaws.isEmpty()) return null;

    double medianYaw = getMedianOfList(recentVisionYaws);
    VisionMeasurement MT2 = getVisionBotPoseMT2();
    Pose2d MT2pose = MT2.getRobotPosition();
    double MT2timestamp = MT2.getTimestamp();
    
    if (MT2pose == null) return null;
    
    return new VisionMeasurement(new Pose2d(MT2pose.getTranslation(), Rotation2d.fromDegrees(medianYaw)),
    MT2timestamp);
  }

  public double getTagId() {
    double leftTag = getTagId(leftLimelightName);
    double rightTag = getTagId(rightLimelightName);
    
    LimelightHelpers.PoseEstimate left = getPoseEstimateMT1(leftLimelightName);
    LimelightHelpers.PoseEstimate right = getPoseEstimateMT1(rightLimelightName);

    int leftCount = left != null ? left.tagCount : 0;
    int rightCount = right != null ? right.tagCount : 0;
    double leftAmb = left != null ? getMaxRawFiducialAmbiguity(left.rawFiducials) : 0.0;
    double rightAmb = right != null ? getMaxRawFiducialAmbiguity(right.rawFiducials) : 0.0;
    
    String bestLimelight = selectBestLimelightName(
      leftCount, leftAmb,
      rightCount, rightAmb
    );

    if (bestLimelight != null && bestLimelight.equals(leftLimelightName) && leftTag != noTagInSightId) {
      return leftTag;
    }
    if (bestLimelight != null && bestLimelight.equals(rightLimelightName) && rightTag != noTagInSightId) {
      return rightTag;
    }
    if (leftTag != noTagInSightId) return leftTag;
    return rightTag;
  }

  public int getLastFiducialCount() { return lastFiducialCount; }
  public double getLastMaxFiducialAmbiguity() { return lastMaxFiducialAmbiguity; }
  public double getLastHeartbeat() { return lastHeartbeat; }

  public boolean isVisionQualityGood() {
    return lastFiducialCount >= MIN_FIDUCIALS_FOR_VISION
        && lastMaxFiducialAmbiguity <= Constants.TAG_AMBIGUITY_THRESHOLD;
  }

  private void updateFiducialDiagnostics(int leftCount, double leftAmb, int rightCount, double rightAmb) {
    lastFiducialCount = Math.max(leftCount, rightCount);
    lastMaxFiducialAmbiguity = getBestAmbiguity(leftCount, leftAmb, rightCount, rightAmb);
  }

  private void updatePreferredLimelightLEDs(int leftCount, double leftAmb, int rightCount, double rightAmb) {
    String preferred = selectBestLimelightName(leftCount, leftAmb, rightCount, rightAmb);
    if (preferred == null || preferred.equals(lastPreferredLimelight)) {
      return;
    }

    boolean hasTargets = leftCount > 0 || rightCount > 0;
    if (!hasTargets) {
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

  private void updateCameraIMU(Rotation2d yaw){
    LimelightHelpers.SetRobotOrientation(leftLimelightName, yaw.getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(rightLimelightName, yaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  public void clearRecentVisionYaws(){
    recentVisionYaws.clear();
  }

  private Double getMedianOfList(ArrayList<Double> list) {
    ArrayList<Double> modifiedList = new ArrayList<Double>(list);
    Collections.sort(modifiedList);
    return modifiedList.get((int) (modifiedList.size() / 2));
  }

  public Pose2d getVisionBotPoseInTargetSpace() {
    LimelightHelpers.PoseEstimate left = getPoseEstimateMT1(leftLimelightName);
    LimelightHelpers.PoseEstimate right = getPoseEstimateMT1(rightLimelightName);

    int leftCount = left != null ? left.tagCount : 0;
    int rightCount = right != null ? right.tagCount : 0;
    double leftAmb = left != null ? getMaxRawFiducialAmbiguity(left.rawFiducials) : 0.0;
    double rightAmb = right != null ? getMaxRawFiducialAmbiguity(right.rawFiducials) : 0.0;
    
    String limelightName = selectBestLimelightName(
      leftCount, leftAmb,
      rightCount, rightAmb
    );
    return getVisionBotPoseInTargetSpace(limelightName);
  }

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

  public static Pose2d translateLimelightPoseToWPIBlue(Pose2d LLPose) {
    return new Pose2d(
        LLPose.getTranslation()
            .plus(new Translation2d(Constants.limelightToWPIBlueXOffest, Constants.limelightToWPIBlueYOffset)),
        LLPose.getRotation());
  }

  @Override
  public void periodic() {
  }
}