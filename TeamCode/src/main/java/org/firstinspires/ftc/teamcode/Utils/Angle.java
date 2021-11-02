package org.firstinspires.ftc.teamcode.Utils;

public class Angle {
    public static double getMAngle(double mAngle, double targetAngle) {
        double difference = targetAngle-mAngle;

        if(difference < -180) {
            difference = 360+difference;
        } else if(difference > 180) {
            difference = 360-difference;
            difference *= -1;
        }

        return targetAngle - difference;
    }
}
