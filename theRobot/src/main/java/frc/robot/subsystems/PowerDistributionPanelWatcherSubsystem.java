// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: PowerDistributionPanelWatcherSubsystem.java
// Intent: Forms util class to watch PDP ports for overcrrent.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.PortSpy;
import frc.robot.control.Constants;

public class PowerDistributionPanelWatcherSubsystem extends SubsystemBase {
    private PowerDistribution distroPannel = new PowerDistribution(
            Constants.currentPowerDistributionPanelCanId,
            Constants.currentPowerDistributionPanelType);
    private ArrayList<PortSpy> myList = new ArrayList<PortSpy>();

    private int brownoutEventCount = 0;
    private Command brownoutAction = null;
    private int brownoutEventsBeforeAction;
    private boolean handleBrownouts = false;

    public PowerDistributionPanelWatcherSubsystem() {
        /*
         * DataLogManager.log("CTOR PowerDistributionPanelWatcherSubsystem");
         * for(var next: Thread.currentThread().getStackTrace()) {
         * DataLogManager.log(next.toString());
         * }
         */
        // CommandScheduler.getInstance().registerSubsystem(this);
    }

    /*
     * Method to add new ports to watch for overcurrent protection on
     * 
     * @param spy
     */
    public void add(PortSpy spy) {
        myList.add(spy);
    }

    /**
     * Get the power distro
     * 
     * @return the power distro object
     */
    public PowerDistribution getPowerDistribution() {
        return distroPannel;
    }

    /**
     * enable or disable a port that is currently under watch
     * 
     * @param targetPort   - the port we want to make sure is disabled
     * @param enabledValue - if true the port will be disabled, else if false it
     *                     will be enabled
     */
    public void setEnabledWatchOnPort(int targetPort, boolean enabledValue) {
        for (int counter = 0; counter < myList.size(); counter++) {
            PortSpy nextSpy = myList.get(counter);
            if (nextSpy.getPort() == targetPort) {
                nextSpy.setEnabled(enabledValue);
                break;
            }
        }
    }

    /**
     * Sets the callback action to run once brownoutEventsPerAction brownouts happen
     * 
     * @param brownoutAction
     * @param brownoutEventsBeforeAction
     */
    public void setBrownoutCallback(Command brownoutAction, int brownoutEventsBeforeAction) {
        this.brownoutAction = brownoutAction;
        this.brownoutEventsBeforeAction = brownoutEventsBeforeAction;
        this.handleBrownouts = true;
        this.brownoutEventCount = 0;
    }

    @Override
    public void periodic() {
        // handle brownouts
        handleBrownouts();
        SmartDashboard.putNumber("Brownout Count", this.brownoutEventCount);
        // handle port spies
        for (int counter = 0; counter < myList.size(); counter++) {
            PortSpy nextSpy = myList.get(counter);
            double current = distroPannel.getCurrent(nextSpy.getPort());

            if (nextSpy.getEnabled() && current > nextSpy.getCurrentLimit()) {
                DataLogManager.log(
                        "Overcurrent detected for port " + nextSpy.getPort() +
                                " with maximum of " + nextSpy.getCurrentLimit() +
                                " and actual of " + current +
                                ". -> " + nextSpy.getActionDescription());
                // lanunch the command
                CommandScheduler.getInstance().schedule(nextSpy.getAction());
            }
            SmartDashboard.putNumber(nextSpy.getActionDescription(), current);
        }
    }

    /**
     * A method to check for and handle brownout events
     * runs the brownoutAction once brownoutEventsBeforeAction brownouts are
     * detected.
     */
    private void handleBrownouts() {
        if (isBrownedOut()) {
            brownoutEventCount += 1;
            if (this.handleBrownouts && (brownoutEventCount >= this.brownoutEventsBeforeAction)) {
                brownoutAction.schedule();
                ;
                this.handleBrownouts = false;
            }
        }
    }

    /**
     * A Method to check for brownouts.
     * 
     * @return true when a brownout is detected
     */
    private boolean isBrownedOut() {
        return RobotController.isBrownedOut();
    }

}
