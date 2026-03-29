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

    private static class MatchPeriod {
        public final double startSeconds;
        public final double endSeconds;

        public MatchPeriod(double startSeconds, double endSeconds) {
            this.startSeconds = startSeconds;
            this.endSeconds = endSeconds;
        }
    }

    private static final MatchPeriod[] PERIODS = new MatchPeriod[] {
        new MatchPeriod(150.0, 130.0),
        new MatchPeriod(140.0, 130.0),
        new MatchPeriod(130.0, 105.0),
        new MatchPeriod(105.0, 80.0),
        new MatchPeriod(80.0, 55.0),
        new MatchPeriod(55.0, 30.0),
        new MatchPeriod(30.0, 0.0),
    };

    /**
     * Returns 1 if the match clock is within 10-5 seconds before the start of any
     * defined period.
     * Returns 2 if within 5-0 seconds before the start of any defined period.
     * Returns 3 otherwise.
     */
    public static int getPeriodWarningState() {
        double matchTime = DriverStation.getMatchTime();
        for (MatchPeriod period : PERIODS) {
            // Only act during the period (inclusive range) and evaluate the final 10/5 seconds
            if (matchTime <= period.startSeconds && matchTime >= period.endSeconds) {
                double secondsUntilEnd = matchTime - period.endSeconds;
                // Return 4 during the entire last period (30-0)
                if (period.startSeconds == 30.0 && period.endSeconds == 0.0) {
                    return 4;
                }
                // Special-case: second to last period (55-30) 5-second warning should return 4
                if (period.startSeconds == 55.0 && period.endSeconds == 30.0 && secondsUntilEnd <= 5.0) {
                    return 4;
                }
                if (secondsUntilEnd <= 5.0) {
                    return 2;
                }
                if (secondsUntilEnd <= 10.0) {
                    return 1;
                }
            }
        }
        return 3;
    }

    /**
     * Returns 1 for 10-second warning before each shift, 2 for 5-second warning,
     * and 3 for all other times.
     */
    public static int getShiftWarningState() {
        if (isTenTillShift()) {
            return 1;
        }
        if (isFiveTillShift()) {
            return 2;
        }
        return 3;
    }
}
