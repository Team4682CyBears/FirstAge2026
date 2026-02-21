// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: CameraSubsystem.java
// Intent: Forms the preliminary code for ball dectection with the limelight
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class BallDetectionSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public BallDetectionSubsystem() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Ball detected", BallDetectionSubsystem.ballDetected("limelight"));
    System.out.println(BallDetectionSubsystem.ballDetected("limelight"));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public static boolean isBallInCenterZone(String cameraName) {
      // 1. Check if the Limelight actually sees a target
      boolean hasTarget = LimelightHelpers.getTV(cameraName);

      if (hasTarget) {
          double tx = LimelightHelpers.getTX(cameraName); // Horizontal degrees
          double ty = LimelightHelpers.getTY(cameraName); // Vertical degrees
          // 5 pixels at 320x240 resolution is roughly 1.0 degree
          double degreeThreshold = 1.0; 
          // Check if absolute values of tx and ty are within the threshold
          return (Math.abs(tx) <= degreeThreshold) && (Math.abs(ty) <= degreeThreshold);
      }
    return false;
  }
  public static boolean ballDetected(String name){
    if(LimelightHelpers.getTV(name) == true){
      return true;
    }
    return false;
  }
}


