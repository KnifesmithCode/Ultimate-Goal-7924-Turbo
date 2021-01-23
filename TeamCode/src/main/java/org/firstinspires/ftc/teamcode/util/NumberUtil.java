package org.firstinspires.ftc.teamcode.util;

public class NumberUtil {
    /**
     * Return the maximum of an array of doubles
     *
     * @param numbers The array of doubles
     * @return The maximum value in the array
     */
    public static double max(double[] numbers) {
        double max = 0;
        for (double number : numbers) {
            if (number > max) max = number;
        }
        return max;
    }
}
