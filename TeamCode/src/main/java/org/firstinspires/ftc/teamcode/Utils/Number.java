package org.firstinspires.ftc.teamcode.Utils;

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
}
