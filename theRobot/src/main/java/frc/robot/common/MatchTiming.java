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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Constants;

public class MatchTiming {
    public MatchTiming(){
    }

    //Returns 'B' if Blue starts and 'R' if red starts
    public String startingHub(){
        String gameData;
        gameData = DriverStation.getGameSpecificMessage();
        return gameData;
    }

    /** returns a boolean saying if your hub is active */
    public boolean isHubActive(){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if(alliance.isEmpty()){
            return false;
        }

        if(!DriverStation.isTeleopEnabled()){
            return false;
        }

        double matchTime = DriverStation.getMatchTime();
        String firstHubActive = startingHub();

        if (firstHubActive.isEmpty()){
            return true;
        }

        boolean redInactiveFirst = false;
        switch (firstHubActive.charAt(0)){
            case 'R' -> redInactiveFirst = true;
            case 'B' -> redInactiveFirst = false;
            default -> {return true;}
        }

        boolean shift1Active = switch (alliance.get()){
            case Red -> !redInactiveFirst;
            case Blue ->redInactiveFirst;
        };

        if (matchTime > 130){
            return true;
        }else if (matchTime > 105){
            return shift1Active;
        }else if (matchTime > 80){
            return !shift1Active;
        }else if (matchTime > 55){
            return shift1Active;
        }else if (matchTime > 30){
            return !shift1Active;
        }else{
            return true;
        }
    }

    public static boolean isFiveTillMajorShift(){
        double matchTime = DriverStation.getMatchTime();

        if(matchTime < Constants.autoTimeSeconds && matchTime > 25){
            return true;
        }else if(matchTime < Constants.endGameStartSeconds + 5 && matchTime > Constants.endGameStartSeconds){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isNewShift(){
        double matchTime = DriverStation.getMatchTime();
        double remainder = (matchTime - Constants.autoTimeSeconds)%Constants.shiftDurationSeconds;

        if(remainder < 15 && matchTime < Constants.endGameStartSeconds && matchTime > Constants.autoTimeSeconds){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isTenTillShift(){
        double matchTime = DriverStation.getMatchTime();
        double remainder = (matchTime - Constants.autoTimeSeconds)%Constants.shiftDurationSeconds;

        if(remainder < 20 && remainder >= 15 && !isEndGame() && matchTime > Constants.autoTimeSeconds){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isFiveTillShift(){
        double matchTime = DriverStation.getMatchTime();
        double remainder = (matchTime - Constants.autoTimeSeconds)%Constants.shiftDurationSeconds;

        if(remainder >= 20 && !isEndGame() && !isAuto()){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isEndOrAuto(){
        double matchTime = DriverStation.getMatchTime();
        if(matchTime < Constants.autoTimeSeconds - 5 || Constants.matchTime > 130){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isEndGame(){
        double matchTime = DriverStation.getMatchTime();
        if(matchTime >= Constants.endGameStartSeconds){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isAuto(){
        double matchTime = DriverStation.getMatchTime();
        if(matchTime >= Constants.autoTimeSeconds){
            return true;
        }else{
            return false;
        }
    }
}

