// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: DrivetrainPowerSubsystem.java
// Intent: Forms a subsystem to control ordered access to writeable variables in DrivetrainSubsystem without requring access there.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.MotorUtils;

public class DrivetrainPowerSubsystem extends SubsystemBase {

    private DrivetrainSubsystem currentDrivetrainSubsystem = null;

    private final double maximumSpeedReductionFactor = 1.0;
    private final double defaultSpeedReductionFactor = 1.0;
    private final double reducedSpeedReductionFactor = 0.5; // used for fine control
    private double speedReductionFactor = defaultSpeedReductionFactor;
    private double speedReductionFactorIncrement = 0.1;

    /**
     * Subsystem that will help coordinate access to the DrivetrainSubsystem
     * 
     * @param currentDrivetrain -
     */
    public DrivetrainPowerSubsystem(DrivetrainSubsystem currentDrivetrain) {
        currentDrivetrainSubsystem = currentDrivetrain;
        this.speedReductionFactor = currentDrivetrainSubsystem.getSpeedReductionFactor();
    }

    /**
     * Method to decrement the power reduction factor
     */
    public void decrementPowerReductionFactor() {
        speedReductionFactor = MotorUtils.truncateValue(
                speedReductionFactor - speedReductionFactorIncrement,
                speedReductionFactorIncrement,
                maximumSpeedReductionFactor);
        this.updateSpeedReductionFactor();
    }

    /**
     * A method to decrement the power reduction factor
     * with a specified minimum
     * 
     * @param minimum
     * @return true if minimum has been reached
     */
    public boolean decrementPowerReductionFactor(double minimum) {
        speedReductionFactor = MotorUtils.truncateValue(
                speedReductionFactor - speedReductionFactorIncrement,
                Math.max(minimum, speedReductionFactorIncrement),
                maximumSpeedReductionFactor);
        this.updateSpeedReductionFactor();
        return speedReductionFactor == Math.max(minimum, speedReductionFactorIncrement);
    }

    /**
     * A method to return the reduced speed reduction factor
     * 
     * @return
     */
    public double getReducedSpeedReductionFactor() {
        return reducedSpeedReductionFactor;
    }

    /**
     * Method to increment the power reduction factor
     */
    public void incrementPowerReductionFactor() {
        speedReductionFactor = MotorUtils.truncateValue(
                speedReductionFactor + speedReductionFactorIncrement,
                speedReductionFactorIncrement,
                maximumSpeedReductionFactor);
        this.updateSpeedReductionFactor();
    }

    /**
     * A method to determine if the power reduction factor is maximum
     * 
     * @return true if is maximum, false otherwise
     */
    public boolean isMaxPowerReductionFactor() {
        return speedReductionFactor == maximumSpeedReductionFactor;
    }

    /**
     * Method to reset the power reduction factor
     */
    public void resetPowerReductionFactor() {
        speedReductionFactor = defaultSpeedReductionFactor;
        this.updateSpeedReductionFactor();
    }

    public void setReducedPowerReductionFactor() {
        speedReductionFactor = reducedSpeedReductionFactor;
        this.updateSpeedReductionFactor();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void updateSpeedReductionFactor() {
        if (this.currentDrivetrainSubsystem != null) {
            this.currentDrivetrainSubsystem.setSpeedReductionFactor(this.speedReductionFactor);
        }
    }
}
