// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: MatchTiming.java
// Intent: a class to handle all matchtiming functionality
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.common;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.control.Constants;

public class MatchTiming {

    // Constructor
    public MatchTiming() {
    }

    // Currently unused enum for when the hub is active will try to get working
    // eventually
    public enum hubActiveLogic {
        RedFirst,
        BlueFirst,
        Unknown,
    }

    // Returns 'B' if Blue starts and 'R' if red starts
    public String startingHub() {
        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        return gameData;
    }

    /** returns a boolean saying if your hub is active */
    public boolean isHubActive() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            return false;
        }

        if (!DriverStation.isTeleopEnabled()) {
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String firstHubActive = startingHub();

        if (firstHubActive.isEmpty()) {
            return true;
        }

        boolean redInactiveFirst = false;
        switch (firstHubActive.charAt(0)) {
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {
                return true;
            }
        }

        boolean shift1Active = switch (alliance.get()) {
            case Red -> !redInactiveFirst;
            case Blue -> redInactiveFirst;
        };

        if (matchTime > 130) {
            return true;
        } else if (matchTime > 105) {
            return shift1Active;
        } else if (matchTime > 80) {
            return !shift1Active;
        } else if (matchTime > 55) {
            return shift1Active;
        } else if (matchTime > 30) {
            return !shift1Active;
        } else {
            return true;
        }
    }

    // Returns true if there are five seconds till the end of auto or until endgame
    public static boolean isFiveTillMajorShift() {
        double matchTime = DriverStation.getMatchTime();

        if (matchTime < Constants.autoTimeSeconds && matchTime > 25) {
            return true;
        } else if (matchTime < Constants.endGameStartSeconds + 5 && isEndGame()) {
            return true;
        } else {
            return false;
        }
    }

    // Returns true for the first 15 seconds of each shift
    public static boolean isNewShift() {
        double matchTime = DriverStation.getMatchTime();
        double remainder = (matchTime - Constants.autoTimeSeconds) % Constants.shiftDurationSeconds;

        if (remainder < 15 && isTeleopShifts()) {
            return true;
        } else {
            return false;
        }
    }

    // Returns true when there are 10-5 seconds left till the next shift
    public static boolean isTenTillShift() {
        double matchTime = DriverStation.getMatchTime();
        double remainder = (matchTime - Constants.autoTimeSeconds) % Constants.shiftDurationSeconds;

        if (remainder < 20 && remainder >= 15 && isTeleopShifts()) {
            return true;
        } else {
            return false;
        }
    }

    // Returns true when there are 5 seconds till the next shift
    public static boolean isFiveTillShift() {
        double matchTime = DriverStation.getMatchTime();
        double remainder = (matchTime - Constants.autoTimeSeconds) % Constants.shiftDurationSeconds;

        if (remainder >= 20 && isTeleopShifts()) {
            return true;
        } else {
            return false;
        }
    }

    // Returns true if it is endgame or auto
    public static boolean isEndOrAuto() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime < Constants.autoTimeSeconds - 5 || matchTime > 130) {
            return true;
        } else {
            return false;
        }
    }

    // Returns true if it is 5 seconds before end game up till the end of the match
    public static boolean isEndGame() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime >= Constants.endGameStartSeconds) {
            return true;
        } else {
            return false;
        }
    }

    // returns true if it is auto
    public static boolean isAuto() {
        double matchTime = DriverStation.getMatchTime();
        if (matchTime >= Constants.autoTimeSeconds) {
            return true;
        } else {
            return false;
        }
    }

    // Returns true if it is neither auto or 5 seconds before end game
    public static boolean isTeleopShifts() {
        if (!isAuto() && !isEndGame()) {
            return true;
        } else {
            return false;
        }
    }
}
