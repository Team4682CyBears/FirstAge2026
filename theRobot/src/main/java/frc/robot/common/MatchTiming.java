package frc.robot.common;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    /**Returns how much time is left in the shift as a double 
     * returns 72 if in auto and returns 4682 if in endgame
     * I can improve on how it handles those "edge" cases as
     * I learn better what we want the final implementation to do
    */
    public double timeLeftInShift(){
        
        double matchTime = DriverStation.getMatchTime();

        if(matchTime < 30){
            return 72;
            /**this could defintiely be done better but it is 
              * basically just to tell you that you are in auto and not 
              * something else because the time will never be over 25 otherwise
            **/
        }else if(matchTime < 130){
            double remainder = (matchTime - 30)%25;
            return remainder;
        }else{
            return 4682;
            /** same idea as above */
        }
    }
}

