// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rebuilt - 2026
// File: MotorUtils.java
// Intent: Forms util class of methods that are commonly used on motor input/output.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.signum;
import static java.lang.Math.abs;

public class MotorUtils {

    /**
     * a method to validate speed input and throw if the speed value is invalid
     *
     * @param speed          - target speed of the motor
     * @param prependMessage - when speed is invalid, a string added to the front of
     *                       the exception message
     * @param appendMessage  - when speed is invalid, a string added to the end of
     *                       the exception message
     */
    public static void validateMotorSpeedInput(
            double speed,
            String prependMessage,
            String appendMessage) {
        if (speed > 1.0 || speed < -1.0) {
            throw new IllegalArgumentException(
                    prependMessage == null ? ""
                            : prependMessage +
                                    "input outside of acceptable motor speed range (valid range from -1.0 to 1.0)" +
                                    appendMessage == null ? "" : appendMessage);
        }
    }

    /**
     * A method to make sure that values are retained within the boundaries
     *
     * @param value       - target value
     * @param minBoundary - when value is below the minimum boundary the minBoundary
     *                    is used as the returned value
     * @param maxBoundary - when value is above the maximum boundary the maxBoundary
     *                    is used as the returned value
     * @return the target or trimmed value
     */
    public static double truncateValue(
            double value,
            double minBoundary,
            double maxBoundary) {
        double trimmedValue = value;
        if (value < minBoundary) {
            trimmedValue = minBoundary;
        } else if (value > maxBoundary) {
            trimmedValue = maxBoundary;
        }
        return trimmedValue;
    }

    /**
     * Clamps a value between a minimum and a maximum value.
     *
     * @param value The value to clamp.
     * @param min   The minimum value of the range. This value must be less than
     *              max.
     * @param max   The maximum value of the range. This value must be greater than
     *              min.
     * @return the clamped value.
     *
     */
    public static double clamp(double value, double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min must not be greater than max");
        }

        return max(min, min(value, max));
    }

    /**
     * Clamps absolute value between a minimum and a maximum value. Retains the
     * sign.
     * 0 returns 0
     *
     * @param value The value to clamp.
     * @param min   The minimum value of the range. This value must be positive and
     *              less than max.
     * @param max   The maximum value of the range. This value must be positive and
     *              greater than min.
     * @return the clamped value.
     *
     */
    public static double doubleSidedClamp(double value, double min, double max) {
        if (min > max) {
            throw new IllegalArgumentException("min must not be greater than max");
        }
        if (min < 0) {
            throw new IllegalArgumentException("min must be positive");
        }

        double sign = signum(value);
        // sign returns {-1, 0, 1},
        // so below returns 0 when input value = 0, and the clamped value otherwise;
        return sign * clamp(abs(value), min, max);
    }

}
