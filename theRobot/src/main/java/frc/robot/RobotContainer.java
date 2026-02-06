// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: RobotContainer.java
// Intent: main robot body
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.common.TestTrajectories;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.control.AutonomousChooser;
import frc.robot.control.Constants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();
  private AutonomousChooser autonomousChooser;

  public RobotContainer() {

    // init the data logging
    this.initializeDataLogging();

    // init the pdp watcher
    this.initializePowerDistributionPanelWatcherSubsystem();

    // init the camera (before drivetrain)
    this.initializeCameraSubsystem();

    // init the leds
    this.initializeLEDSubsystem();

    // init the shooter subsystem
    this.initializeShooterSubsystem();

    // init the hood subsystem
    this.initializeHoodSubsystem();

    // init the kicker subsystem
    this.initializeKickerSubsystem();

    // init the various subsystems
    this.initializeDrivetrainSubsystem();

    // init the shot logger (after drivetrain, shooter and hood are initialized)
    this.initializeShotLogger();

    // init the input system
    this.initializeManualInputInterfaces();

    // do late binding of default commands
    this.lateBindDefaultCommands();

    if (subsystems.isDriveTrainSubsystemAvailable()) {
      AutonomousChooser.configureAutoBuilder(subsystems);
      autonomousChooser = new AutonomousChooser(subsystems);
      DataLogManager.log("SUCCESS: initializeAutoChooser");
    } else {
      DataLogManager.log("FAIL: initializeAutoChooser");
    }

    // Configure the button bindings
    if (this.subsystems.isManualInputInterfacesAvailable()) {
      DataLogManager.log(">>>> Initializing button bindings.");
      this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
      DataLogManager.log(">>>> Finished initializing button bindings.");
    }

    // TODO For debugging. Can remove for final competition build.
    this.initializeDebugDashboard();

    if (subsystems.isDriveTrainSubsystemAvailable() && Constants.putDiagnosticPaths) {
      // Path Planner Path Commands
      // commands to drive path planner test trajectories
      TestTrajectories testtrajectories = new TestTrajectories();

      SmartDashboard.putData("One Meter",
          FollowTrajectoryCommandBuilder.build(testtrajectories.oneMeter, this.subsystems.getDriveTrainSubsystem()));
      SmartDashboard.putData("Two Meter",
          FollowTrajectoryCommandBuilder.build(testtrajectories.twoMeter, this.subsystems.getDriveTrainSubsystem()));
      SmartDashboard.putData("Three Meter",
          FollowTrajectoryCommandBuilder.build(testtrajectories.threeMeter, this.subsystems.getDriveTrainSubsystem()));
    }
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }

  /**
   * A method to init all the data logging
   */
  private void initializeDataLogging() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * A method to init the PDP watcher
   */
  private void initializePowerDistributionPanelWatcherSubsystem() {
    if (InstalledHardware.powerDistributionPanelInstalled) {
      subsystems.setPowerDistributionPanelWatcherSubsystem(new PowerDistributionPanelWatcherSubsystem());
      DataLogManager.log("SUCCESS: initializePowerDistributionPanelWatcherSubsystem");
    } else {
      DataLogManager.log("FAIL: initializePowerDistributionPanelWatcherSubsystem");
    }
  }

  /**
   * A method to init the drive train
   */
  private void initializeDrivetrainSubsystem() {
    if (InstalledHardware.drivetrainInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setDriveTrainSubsystem(new DrivetrainSubsystem(subsystems));
      subsystems.setDriveTrainPowerSubsystem(new DrivetrainPowerSubsystem(subsystems.getDriveTrainSubsystem()));
      DataLogManager.log("SUCCESS: initializeDrivetrain");

      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation
      subsystems.getDriveTrainSubsystem().setDefaultCommand(
          new DefaultDriveCommand(
              subsystems.getDriveTrainSubsystem(),
              () -> RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveY())
                  * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveX())
                  * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
              () -> -RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputSpinDriveX())
                  * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    } else {
      DataLogManager.log("FAIL: initializeDrivetrain");
    }
  }

  /**
   * A method to init items for the debug dashboard
   */
  private void initializeDebugDashboard() {
    SmartDashboard.putData("Debug: CommandScheduler", CommandScheduler.getInstance());
  }

  /**
   * A method to init the CameraSubsystem
   */
  private void initializeCameraSubsystem() {
    if (InstalledHardware.limelightInstalled) {
      subsystems.setCameraSubsystem(new CameraSubsystem());
      DataLogManager.log("SUCCESS: initializeCamera");
    } else {
      DataLogManager.log("FAIL: initializeCamera");
    }
  }

  /**
   * A method to init the LEDSubsystem
   */
  private void initializeLEDSubsystem() {
    if (InstalledHardware.LEDSInstalled) {
      subsystems.setLEDSubsystem(new LEDSubsystem(Constants.ledPWMID));
      System.out.println("SUCCESS: initializeLEDS");
    } else {
      System.out.println("FAIL: initializeLEDS");
    }
  }

  /**
   * A method to init the shooter subsystem
   */
  private void initializeShooterSubsystem() {
    if (InstalledHardware.shooterInstalled) {
      subsystems.setShooterSubsystem(
          new ShooterSubsystem(Constants.shooterLeadMotorCanId, Constants.shooterFollowMotorCanId));
      System.out.println("SUCCESS: initializeShooter");
    } else {
      System.out.println("FAIL: initializeShooter");
    }
  }

  /**
   * A method to init the hood subsystem
   */
  private void initializeHoodSubsystem() {
    if (InstalledHardware.hoodInstalled) {
      subsystems.setHoodSubsystem(new HoodSubsystem(Constants.hoodServoMotorCanId));
      System.out.println("SUCCESS: initializeHood");
    } else {
      System.out.println("FAIL: initializeHood");
    }
  }

  /**
   * A method to init the hood subsystem
   */
  private void initializeKickerSubsystem() {
    if (InstalledHardware.kickerInstalled) {
      subsystems.setKickerSubsystem(new KickerSubsystem());
      System.out.println("SUCCESS: initializeKicker");
    } else {
      System.out.println("FAIL: initializeKicker");
    }
  }

  /**
   * A method to init the input interfaces
   */
  private void initializeManualInputInterfaces() {
    // note: in this case it is safe to build the interfaces if only one of the
    // controllers is present
    // because button binding assignment code checks that each is installed later
    // (see: initializeButtonCommandBindings)
    if (InstalledHardware.driverXboxControllerInstalled ||
        InstalledHardware.coDriverXboxControllerInstalled) {
      subsystems.setManualInputInterfaces(new ManualInputInterfaces(subsystems));
      DataLogManager.log("SUCCESS: initializeManualInputInterfaces");
    } else {
      DataLogManager.log("FAIL: initializeManualInputInterfaces");
    }
  }

  /**
   * Initialize the shot logger helper that writes shot events to disk.
   */
  private void initializeShotLogger() {
    // Require at minimum drivetrain and shooter/hood to be available to make logs
    // useful
    if (subsystems.isDriveTrainSubsystemAvailable() && subsystems.isShooterSubsystemAvailable()
        && subsystems.isHoodSubsystemAvailable()) {
      subsystems.setShotLogger(new frc.robot.subsystems.ShotLogger(subsystems));
      DataLogManager.log("SUCCESS: initializeShotLogger");
    } else {
      DataLogManager.log("FAIL: initializeShotLogger - missing subsystems");
    }
  }

  /**
   * A method to late binding of default commands
   */
  private void lateBindDefaultCommands() {
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  private static double modifyAxisSquare(double value) {
    // Deadband
    value = deadband(value, 0.065);

    // Joystick input exponent
    value = Math.copySign(value * value, value);

    return value;
  }
}
