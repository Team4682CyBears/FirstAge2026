// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.KickerSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShotLogger;
import frc.robot.subsystems.SpindexerSpinner;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeRollerSubsystem;
import frc.robot.subsystems.IntakeWristSubsystem;

public class SubsystemCollection {
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems
    private CameraSubsystem cameraSubsystem = null;
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null;
    private LEDSubsystem ledSubsystem = null;
    private ShooterSubsystem shooterSubsystem = null;
    private KickerSubsystem kickerSubsystem = null;
    private HoodSubsystem hoodSubsystem = null;
    private ClimberSubsystem climberSubsystem = null;
    private IntakeRollerSubsystem intakeRollerSubsystem = null;
    private IntakeWristSubsystem intakeWristSubsystem = null;
    private ShotLogger shotLogger = null;
    private SpindexerSpinner spindexer = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {
    }

    public DrivetrainSubsystem getDriveTrainSubsystem() {
        return driveTrainSubsystem;
    }

    public boolean isDriveTrainSubsystemAvailable() {
        return driveTrainSubsystem != null;
    }

    public void setDriveTrainSubsystem(DrivetrainSubsystem value) {
        driveTrainSubsystem = value;
    }

    public CameraSubsystem getCameraSubsystem() {
        return cameraSubsystem;
    }

    public void setCameraSubsystem(CameraSubsystem value) {
        cameraSubsystem = value;
    }

    public boolean isCameraSubsystemAvailable() {
        return cameraSubsystem != null;
    }

    public LEDSubsystem getLedSubsystem() {
        return ledSubsystem;
    }

    public void setLEDSubsystem(LEDSubsystem value) {
        ledSubsystem = value;
    }

    public boolean isLEDSubsystemAvailable() {
        return ledSubsystem != null;
    }

    public ShooterSubsystem getShooterSubsystem() {
        return shooterSubsystem;
    }

    public void setShooterSubsystem(ShooterSubsystem value) {
        shooterSubsystem = value;
    }

    public boolean isShooterSubsystemAvailable() {
        return shooterSubsystem != null;
    }

    public HoodSubsystem getHoodSubsystem() {
        return hoodSubsystem;
    }

    public void setHoodSubsystem(HoodSubsystem value) {
        hoodSubsystem = value;
    }

    public boolean isHoodSubsystemAvailable() {
        return hoodSubsystem != null;
    }

    public ClimberSubsystem getClimberSubsystem() {
        return climberSubsystem;
    }

    public void setClimberSubsystem(ClimberSubsystem value) {
        climberSubsystem = value;
    }

    public boolean isClimberSubsystemAvailable() {
        return climberSubsystem != null;
    }

    public KickerSubsystem getKickerSubsystem() {
        return kickerSubsystem;
    }

    public void setKickerSubsystem(KickerSubsystem value) {
        kickerSubsystem = value;
    }

    public boolean isKickerSubsystemAvailable() {
        return kickerSubsystem != null;
    }

    public IntakeRollerSubsystem getIntakeRollerSubsystem() {
        return intakeRollerSubsystem;
    }

    public void setIntakeRollerSubsystem(IntakeRollerSubsystem value) {
        intakeRollerSubsystem = value;
    }

    public boolean isIntakeRollerSubsystemAvailable() {
        return intakeRollerSubsystem != null;
    }

    public IntakeWristSubsystem getIntakeWristSubsystem() {
        return intakeWristSubsystem;
    }

    public void setIntakeWristSubsystem(IntakeWristSubsystem value) {
        intakeWristSubsystem = value;
    }

    public boolean isIntakeWristSubsystemAvailable() {
        return intakeWristSubsystem != null;
    }

    public ShotLogger getShotLogger() {
        return shotLogger;
    }

    public void setShotLogger(ShotLogger value) {
        shotLogger = value;
    }

    public boolean isShotLoggerAvailable() {
        return shotLogger != null;
    }

    public DrivetrainPowerSubsystem getDriveTrainPowerSubsystem() {
        return driveTrainPowerSubsystem;
    }

    public void setDriveTrainPowerSubsystem(DrivetrainPowerSubsystem value) {
        driveTrainPowerSubsystem = value;
    }

    public boolean isDriveTrainPowerSubsystemAvailable() {
        return driveTrainPowerSubsystem != null;
    }

    public PowerDistributionPanelWatcherSubsystem getPowerDistributionPanelWatcherSubsystem() {
        return powerDistributionPanelWatcherSubsystem;
    }

    public void setPowerDistributionPanelWatcherSubsystem(PowerDistributionPanelWatcherSubsystem value) {
        powerDistributionPanelWatcherSubsystem = value;
    }

    public boolean isPowerDistributionPanelWatcherSubsystemAvailable() {
        return powerDistributionPanelWatcherSubsystem != null;
    }

    public ManualInputInterfaces getManualInputInterfaces() {
        return manualInput;
    }

    public void setManualInputInterfaces(ManualInputInterfaces value) {
        manualInput = value;
    }

    public boolean isManualInputInterfacesAvailable() {
        return manualInput != null;
    }

    public SpindexerSpinner getSpindexerSpinnerSubsystem(){
        return spindexer;
    }

    public void setSpindexerSpinnerSubsystem(SpindexerSpinner spindexer){
        this.spindexer = spindexer;
    }

    public boolean isSpinnerSpindexerSubsystemAvaible(){
        return spindexer != null;
    }
}
