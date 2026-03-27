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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.common.IntakeDirection;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.*;
import frc.robot.subsystems.TurretSubsystem;

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
     * A method to return the driver controller for rumble needs
     * 
     * @return
     */
    public final XboxController getDriverController() {
        return driverControllerForRumbleOnly;
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

                // start button used to temporarily enable camera seeding
                // while the button is held down. We use onTrue to set the
                // flag when pressed and onFalse to clear it when released.
                this.driverController.start().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> {
                                    if (this.subsystemCollection.isDriveTrainSubsystemAvailable()) {
                                        this.subsystemCollection.getDriveTrainSubsystem()
                                                .setSeedingCamera(true);
                                    }
                                }),
                                new ButtonPressCommand(
                                        "driverController.start()",
                                        "enable camera seeding")))
                        .onFalse(
                                new ParallelCommandGroup(
                                        new InstantCommand(() -> {
                                            if (this.subsystemCollection.isDriveTrainSubsystemAvailable()) {
                                                this.subsystemCollection.getDriveTrainSubsystem()
                                                        .setSeedingCamera(false);
                                            }
                                        }),
                                        new ButtonPressCommand(
                                                "driverController.start()",
                                                "disable camera seeding")));

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

            ShooterAimer shooterAimer = getActiveShooterAimer();
            if (this.subsystemCollection.isDriveTrainPowerSubsystemAvailable()
                    && this.subsystemCollection.isHoodSubsystemAvailable()
                    && this.subsystemCollection.isShooterSubsystemAvailable()
                    && shooterAimer != null) {
        this.driverController.y().whileTrue(
            new AutoAimCommand(subsystemCollection, shooterAimer));
            }
        }

        if (this.subsystemCollection.isKickerSubsystemAvailable()
                && this.subsystemCollection.isSpinnerSpindexerSubsystemAvaible()
                && this.subsystemCollection.isIntakeWristSubsystemAvailable()) {
            this.driverController.rightTrigger().whileTrue(new KickerSpindexerAgitateCommand(
                    this.subsystemCollection.getKickerSubsystem(),
                    this.subsystemCollection.getSpindexerSpinnerSubsystem(),
                    this.subsystemCollection.getIntakeWristSubsystem(),
                    this.subsystemCollection.getIntakeRollerSubsystem()));
        }

        // Driver B toggles intake deploy/retract and runs/stops roller while deployed
        if (this.subsystemCollection.isIntakeWristSubsystemAvailable()
                && this.subsystemCollection.isIntakeRollerSubsystemAvailable()) {
            this.driverController.b().onTrue(new ToggleIntakeDeployCommand(
                    this.subsystemCollection.getIntakeWristSubsystem(),
                    this.subsystemCollection.getIntakeRollerSubsystem()));
        }

        if (this.subsystemCollection.isShooterSubsystemAvailable()
                && this.subsystemCollection.isHoodSubsystemAvailable()
                && this.subsystemCollection.isKickerSubsystemAvailable()) {
            this.driverController.rightBumper().whileTrue(new ShooterManualCommand(subsystemCollection));
        }

        if (this.subsystemCollection.isTurretSubsystemAvailable()) {
            TurretSubsystem turret = this.subsystemCollection.getTurretSubsystem();
            this.driverController.povUp().onTrue(
                    new TurretTestPositionCommand(turret, Math.toRadians(90)));
            this.driverController.povRight().onTrue(
                    new TurretTestPositionCommand(turret, Math.toRadians(180)));
            this.driverController.povDown().onTrue(
                    new TurretTestPositionCommand(turret, Math.toRadians(270)));
            this.driverController.povLeft().onTrue(
                    new TurretTestPositionCommand(turret, Math.toRadians(0)));
        }
    }

    private ShooterAimer getActiveShooterAimer() {
        if (this.subsystemCollection.isShooterAimerAvailable()) {
            return this.subsystemCollection.getShooterAimer();
        }
        return null;
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

            if (this.subsystemCollection.isClimberSubsystemAvailable()) {
                this.coDriverController.leftBumper()
                        .onTrue(new ClimberPositionCommand(this.subsystemCollection.getClimberSubsystem(),
                                () -> ClimberSubsystem.MIN_HEIGHT_ABOVE_FLOOR_INCHES));
                this.coDriverController.rightBumper()
                        .onTrue(new ClimberPositionCommand(this.subsystemCollection.getClimberSubsystem(), () -> 28.0));
                this.coDriverController.axisMagnitudeGreaterThan(XboxController.Axis.kRightY.value, 0.1)
                        .and(this.coDriverController.b())
                        .whileTrue(new ClimberVelocityCommand(this.subsystemCollection.getClimberSubsystem(),
                                () -> this.coDriverController.getRightY()));
            }

            if (this.subsystemCollection.isTurretSubsystemAvailable()) {
                                this.coDriverController.start().onTrue(
                                                new ToggleTurretAimModeCommand(subsystemCollection.getTurretSubsystem()));
            }
            
            // unsure what this should go to
            if (this.subsystemCollection.isIntakeWristSubsystemAvailable()) {
                // if the left y stick has a magnitude greater than 0.1, run the command.
                this.coDriverController.axisMagnitudeGreaterThan(XboxController.Axis.kLeftY.value, 0.1).and(this.coDriverController.b())
                        .whileTrue(new IntakeWristManualCommand(
                                this.subsystemCollection.getIntakeWristSubsystem(),
                                () -> -this.coDriverController.getLeftY()));
            }

            if (this.subsystemCollection.isIntakeRollerSubsystemAvailable()) {
                // when y is pressed, toggle the intake roller manual command
                this.coDriverController.y()
                        .toggleOnTrue(
                                new IntakeRollerManualCommand(this.subsystemCollection.getIntakeRollerSubsystem(),
                                        IntakeDirection.INTAKE));
            }
        }
    }
}
