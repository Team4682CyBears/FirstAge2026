// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: LEDStateAction.java
// Intent: Determines the LED state action
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ  

package frc.robot.common;

import java.util.Iterator;
import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BooleanSupplier;

/**
 * Class to encapsulate the LED state action
 */
public class LEDStateAction {

    private int cacheSize = 5;
    private ArrayDeque<Long> resultCache = new ArrayDeque<Long>();
    private HashMap<Long, Pair<Long, Boolean>> resultMap = new HashMap<Long, Pair<Long, Boolean>>();
    private BooleanSupplier shouldTakeAction;
    private LEDState ledState;

    /**
     * A wrapper class to encapsulate both the target state and the supplier that
     * will indicate if the state is signaled or not
     * 
     * @param ledState         - the state associated with the double supplier
     * @param shouldTakeAction - When true the state should be enabled, when false
     *                         the state is not triggered
     */
    public LEDStateAction(LEDState ledState, BooleanSupplier shouldTakeAction) {
        this.shouldTakeAction = shouldTakeAction;
        this.ledState = ledState;
    }

    /**
     * A wrapper class to encapsulate both the target state and the supplier that
     * will indicate if the state is signaled or not
     * 
     * @param ledState         - the state associated with the double supplier
     * @param shouldTakeAction - When true the state should be enabled, when false
     *                         the state is not triggered
     * @param recentCacheSize  - the size of the cache to use ()
     */
    public LEDStateAction(LEDState ledState, BooleanSupplier shouldTakeAction, int recentCacheSize) {
        this.shouldTakeAction = shouldTakeAction;
        this.ledState = ledState;
        this.cacheSize = recentCacheSize;
    }

    /**
     * Accessor to the most recent state of the underlying LED state action
     * 
     * @return When true the LED should be illuminated, when false the LED should
     *         not be illuminated
     */
    public boolean getCurrentState() {
        long currentTime = System.currentTimeMillis();
        Pair<Long, Boolean> resultValue = null;
        if (this.resultMap.containsKey(currentTime)) {
            resultValue = this.resultMap.get(currentTime);
        } else {
            resultValue = this.getImediateCurrentState(currentTime);
            this.cacheState(resultValue);
        }
        return resultValue.value.booleanValue();
    }

    /**
     * Accessor to the most prevelant state found over a succession of recent state
     * evaluations
     * 
     * @return When true the LED should be illuminated, when false the LED should
     *         not be illuminated
     */
    public boolean getRecentState() {
        // first ensure the current state is cached
        this.getCurrentState();

        // find the recent count of times it has been true and false
        int countTrue = 0;
        int countFalse = 0;
        Iterator<Map.Entry<Long, Pair<Long, Boolean>>> iter = this.resultMap.entrySet().iterator();
        while (iter.hasNext()) {
            Map.Entry<Long, Pair<Long, Boolean>> entry = iter.next();
            if (entry.getValue().value) {
                ++countTrue;
            } else {
                ++countFalse;
            }
        }
        return (countTrue >= countFalse);
    }

    /**
     * Getter method for the state associated with the double supplier
     * 
     * @return - The LED state
     */
    public LEDState getLedState() {
        return this.ledState;
    }

    /**
     * Method to insert and properly prune cache mechanism
     * 
     * @param The state that should be added into the cache
     */
    private void cacheState(Pair<Long, Boolean> state) {
        if (!resultMap.containsKey(state.key)) {
            while (resultCache.size() >= cacheSize) {
                Long keyToRemove = resultCache.removeLast();
                if (resultMap.containsKey(keyToRemove)) {
                    resultMap.remove(keyToRemove);
                }
            }
            resultMap.put(state.key, state);
            resultCache.push(state.key);
        }
    }

    /**
     * Get a fresh evaluation of the current state
     * 
     * @return a pair of the time and state of boolean supplier
     */
    private Pair<Long, Boolean> getImediateCurrentState(long currentTimeMillis) {
        boolean state = shouldTakeAction.getAsBoolean();
        return new Pair<Long, Boolean>(currentTimeMillis, state);
    }
}

/**
 * Inner class to store results in
 */
class Pair<K, V> {
    K key;
    V value;

    Pair(K theKey, V theValue) {
        key = theKey;
        value = theValue;
    }
}
