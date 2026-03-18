// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: LookupTableDouble.java
// Intent: a class to handle lookup tables with double values
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

// This class assumes that the lookup table is ordered and in the format (Input, Output)
// There must be at least 1 entry in the table. 
// if no default value is specified, result will be clamped to min/max values.
// if default value is specified, any query greater or less than table input values will return deafult value
public class LookupTableDouble {
    private final double[][] lookupTable;
    private final double defaultValue;
    private final boolean clampValue;

    public LookupTableDouble(double[][] lutInput, double defaultValue) {
        this.lookupTable = lutInput;
        this.defaultValue = defaultValue;
        this.clampValue = false;
    }

    public LookupTableDouble(double[][] lutInput) {
        this.lookupTable = lutInput;
        this.defaultValue = 0.0; // not used in this case
        this.clampValue = true;
    }

    public double getMaxInput() {
        return lookupTable[lookupTable.length - 1][0];
    }

    public double getMaxOutput() {
        return lookupTable[lookupTable.length - 1][1];
    }

    public double getMinInput() {
        return lookupTable[0][0];
    }

    public double getMinOutput() {
        return lookupTable[0][1];
    }

    public double queryTable(double input) {
        if (input < getMinInput()){
            return clampValue ? getMinOutput() : defaultValue;
        }
        else if (input > getMaxInput()){
            return clampValue ? getMaxOutput() : defaultValue;
        }

        // start by setting all inputs/outputs to the first LUT entry
        double lastInput = lookupTable[0][0];
        double nextInput = lookupTable[0][0];
        double lastOutput = lookupTable[0][1];
        double nextOutput = lookupTable[0][1];

        // else find the lookup table entries that enclose the target input
        for (int i = 0; i < lookupTable.length; i++) {
            lastInput = nextInput;
            lastOutput = nextOutput;
            nextInput = lookupTable[i][0];
            nextOutput = lookupTable[i][1];
            if (input >= lastInput && input <= nextInput) {
                // range found
                break;
            }
        }
        // do interpolation on input to get output
        double numerator = (lastOutput * (nextInput - input)) + (nextOutput * (input - lastInput));
        double denominator = (nextInput - lastInput);
        if (denominator == 0){
            // ill-formed LUT with two entries with identical input values, 
            // would result in divide by 0. WLOG return the lower one
            return lastOutput;
        } else {
            return (numerator / denominator);
        }
    }
}
