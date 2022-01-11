package org.firstinspires.ftc.teamcode.Utils;

/*
number util functions
 */
public class Number {
    public static double[] largest(double[] numbers) {
        double biggest = Double.NEGATIVE_INFINITY;
        double index = -1;
        for(int i = 0; i < numbers.length; i++) {
            double number = numbers[i];
            if(number > biggest) {
                biggest = number;
                index = i;
            }
        }
        return new double[]{biggest, index};
    }

    public static double map(double value, double min, double max, double min2, double max2) {
        return (value-min)/(max-min) * (max2-min2) + min2;
    }
}
