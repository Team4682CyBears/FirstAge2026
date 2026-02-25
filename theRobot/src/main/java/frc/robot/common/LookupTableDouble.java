package frc.robot.common;

// This class assumes that the lookup table is ordered and in the format (Input, Output)
public class LookupTableDouble {
    private final double[][] lookupTable;
    private final double defaultValue;

    public LookupTableDouble(double[][] lutInput, double defaultValue) {
        this.lookupTable = lutInput;
        this.defaultValue = defaultValue;
    }

    public double queryTable(double input) {
        double lastInput = 0.0;
        double nextInput = 0.0;
        double lastOutput = 0.0;
        double nextOutput = 0.0;

        boolean rangeFound = false;

        // see if the lookup table contains entries that enclose the target input
        for (int i = 0; i < lookupTable.length; i++) {
            lastInput = nextInput;
            lastOutput = nextOutput;
            nextInput = lookupTable[i][0];
            nextOutput = lookupTable[i][1];
            if (input >= lastInput && input <= nextInput) {
                rangeFound = true;
                break;
            }
        }

        if (!rangeFound) {
            System.out.println("ERROR: Value '" + input + "' does not exist in the range of lookup table with min of '"
                    + lookupTable[0][0] + "' and a max of '" + lookupTable[lookupTable.length][0] + "'");
            return defaultValue;
        } else {
            // do interpolation on input to get output
            double numerator = (lastOutput * (nextInput - input)) + (nextOutput * (input - lastInput));
            double denominator = (nextInput - lastInput);
            return (numerator / denominator);
        }
    }
}
