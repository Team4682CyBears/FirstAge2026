package frc.robot.common;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class MatchTiming {
    static Timer clock;
    public MatchTiming(){
        clock = new Timer();
        clock.start();
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
        //double matchTime = clock.get();

        if(matchTime < 30 && matchTime > 25){
            return true;
        }else if(matchTime < 130 && matchTime > 125){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isNewShift(){
        double matchTime = DriverStation.getMatchTime();
        //double matchTime = clock.get();
        double remainder = (matchTime - 30)%25;

        if(remainder < 15 && matchTime < 125 && matchTime > 30){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isTenTillShift(){
        double matchTime = DriverStation.getMatchTime();
        //double matchTime = clock.get();
        double remainder = (matchTime - 30)%25;

        if(remainder < 20 && remainder >= 15 && matchTime < 125 && matchTime > 30){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isFiveTillShift(){
        double matchTime = DriverStation.getMatchTime();
        //double matchTime = clock.get();
        double remainder = (matchTime - 30)%25;

        if(remainder >= 20 && matchTime < 125 && matchTime > 30){
            return true;
        }else{
            return false;
        }
    }

    public static boolean isAuto(){
        double matchTime = DriverStation.getMatchTime();
        //double matchTime = clock.get();
        if(matchTime < 25 || matchTime > 130){
            return true;
        }else{
            return false;
        }
    }
}

