// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;

public class ManualInputInterfaces {

    // sets joystick variables to joysticks
    private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController);
    private XboxController driverControllerForRumbleOnly = new XboxController(Constants.portDriverController);
    private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
    private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

    private DigitalInput climberLimSwitch = new DigitalInput(Constants.climberLimSwtichChannel);

    // subsystems needed for inputs
    private SubsystemCollection subsystemCollection = null;

    /**
     * The constructor to build this 'manual input' conduit
     */
    public ManualInputInterfaces(SubsystemCollection currentCollection) {
        subsystemCollection = currentCollection;
    }

    /**
     * A method to return the co driver controller for rumble needs
     * 
     * @return
     */
    public final XboxController getCoDriverController() {
        return coDriverControllerForRumbleOnly;
    }

    /**
     * A method to get the arcade drive X componet being input from humans
     * 
     * @return - a double value associated with the magnitude of the x componet
     */
    public double getInputArcadeDriveX() {
        return -driverController.getLeftX();
    }

    /**
     * A method to get the arcade drive Y componet being input from humans
     * 
     * @return - a double value associated with the magnitude of the y componet
     */
    public double getInputArcadeDriveY() {
        return -driverController.getLeftY();
    }

    /**
     * A method to get the spin drive X componet being input from humans
     * 
     * @return - a double value associated with the magnitude of the x componet
     */
    public double getInputSpinDriveX() {
        return driverController.getRightX();
    }

    /**
     * A method to return the Y value of the left joystick on the co-driver's
     * controller
     * 
     * @return - a double value associated with the magnitude of the y componet
     */
    public double getCoDriverLeftY() {
        return -coDriverController.getLeftY();
    }

    /**
     * Checks if the right trigger on the co-driver's controller is currently being
     * held.
     *
     * @return - a boolean if the right trigger is held
     */
    public boolean isCoDriverRightTriggerHeld() {
        return coDriverController.rightTrigger().getAsBoolean();
    }

    /**
     * A method to return the Y value of the right joystick on the co-driver's
     * controller
     * 
     * @return - a double value associated with the magnitude of the y componet
     */
    public double getCoDriverRightY() {
        return -coDriverController.getRightY();
    }

    /**
     * A method to get if the climber lim switch is pressed
     * 
     * @return - a boolean of wether the lim switch is pressed
     */
    public boolean isCLimberLimSwitchPressed() {
        return climberLimSwitch.get();
    }

    /**
     * A method to initialize various commands to the numerous buttons.
     * Need delayed bindings as some subsystems during testing won't always be
     * there.
     */
    public void initializeButtonCommandBindings() {
        // Configure the driver xbox controller bindings
        if (InstalledHardware.driverXboxControllerInstalled) {
            this.bindCommandsToDriverXboxButtons();
        }

        // Configure the co-driver xbox controller bindings
        if (InstalledHardware.coDriverXboxControllerInstalled) {
            this.bindCommandsToCoDriverXboxButtons();
        }
    }

    /**
     * Will attach commands to the Driver XBox buttons
     */
    private void bindCommandsToDriverXboxButtons() {
        if (InstalledHardware.driverXboxControllerInstalled) {
            if (this.subsystemCollection.isDriveTrainSubsystemAvailable()) {
                // Back button zeros the gyroscope (as in zero yaw)
                this.driverController.back().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        subsystemCollection
                                                .getDriveTrainSubsystem()::zeroGyroscope),
                                new ButtonPressCommand(
                                        "driverController.back()",
                                        "zero gyroscope")));
                DataLogManager.log("FINISHED registering back button to zero gyroscope ... ");

            }

            if (this.subsystemCollection.isDriveTrainPowerSubsystemAvailable()) {
                // Enable pit limiter
                this.driverController.leftTrigger().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()::setReducedPowerReductionFactor,
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()),
                                new ButtonPressCommand(
                                        "driverController.leftTrigger()",
                                        "ramp down to reduced speed")));

                // Disable pit limiter
                this.driverController.leftTrigger().onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()::resetPowerReductionFactor,
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()),
                                new ButtonPressCommand(
                                        "driverController.leftTrigger()",
                                        "ramp up to default speed")));
            }

            // x button press will stop all
            this.driverController.x().onTrue(
                    new ParallelCommandGroup(
                            new AllStopCommand(
                                    this.subsystemCollection),
                            new ButtonPressCommand(
                                    "driverController.x()",
                                    "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")));

            this.driverController.y().onTrue(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> this.subsystemCollection.getDriveTrainSubsystem()
                                    .setSwerveYawMode(SwerveYawMode.AUTO)),
                            new ButtonPressCommand("driverController.y()", "Toggle Swerve Yaw Mode AUTO")));

            this.driverController.y().onFalse(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> this.subsystemCollection.getDriveTrainSubsystem()
                                    .setSwerveYawMode(SwerveYawMode.JOYSTICK)),
                            new ButtonPressCommand("driverController.y()", "Toggle Swerve Yaw Mode JOYSTICK")));
        }
    }

    /**
     * Will attach commands to the Co Driver XBox buttons
     */
    private void bindCommandsToCoDriverXboxButtons() {
        if (InstalledHardware.coDriverXboxControllerInstalled) {
            // x button press will stop all
            this.coDriverController.x().onTrue(
                    new ParallelCommandGroup(
                            new AllStopCommand(
                                    this.subsystemCollection),
                            new ButtonPressCommand(
                                    "coDriverController.x()",
                                    "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")));

            this.coDriverController.povUp().onTrue(new InstantCommand(() -> {
                double currentRPM = SmartDashboard.getNumber("Shooter RPM", 0);
                SmartDashboard.putNumber("Shooter RPM", currentRPM + 50);
            }));
            this.coDriverController.povDown().onTrue(new InstantCommand(() -> {
                double currentRPM = SmartDashboard.getNumber("Shooter RPM", 0);
                SmartDashboard.putNumber("Shooter RPM", currentRPM - 50);
            }));
            this.coDriverController.povLeft().onTrue(new InstantCommand(() -> {
                double hoodExtention = SmartDashboard.getNumber("Hood Angle", 1000);
                if(hoodExtention >= 1050){
                    SmartDashboard.putNumber("Hood Angle", hoodExtention - 50);
                }
            }));
            this.coDriverController.povRight().onTrue(new InstantCommand(() -> {
                double hoodExtention = SmartDashboard.getNumber("Hood Angle", 1000);
                if(hoodExtention <= 1950){
                    SmartDashboard.putNumber("Hood Angle", hoodExtention + 50);
                }
            }));
            this.coDriverController.y().onTrue(new InstantCommand(() -> {
                double hoodExtention = SmartDashboard.getNumber("Hood Extendo", 1000);
                if(hoodExtention >= 1050){
                    SmartDashboard.putNumber("Hood Extendo", hoodExtention - 50);
                }
            }));
            this.coDriverController.b().onTrue(new InstantCommand(() -> {
                double hoodExtention = SmartDashboard.getNumber("Hood Extendo", 1000);
                if(hoodExtention <= 1950){
                    SmartDashboard.putNumber("Hood Extendo", hoodExtention + 50);
                }
            }));

            if (InstalledHardware.shooterInstalled) {
                this.coDriverController.leftTrigger()
                        .whileTrue(new ShootCommand(this.subsystemCollection.getShooterSubsystem(), () -> {
                            return SmartDashboard.getNumber("Shooter RPM", 0);
                        }));
            }
            if (InstalledHardware.hoodInstalled) {
                this.coDriverController.a().onTrue(new HoodAngleCommand(this.subsystemCollection.getHoodSubsystem()));
            }
        }
    }
}
