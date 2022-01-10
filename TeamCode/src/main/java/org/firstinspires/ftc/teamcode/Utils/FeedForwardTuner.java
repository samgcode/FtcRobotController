package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;

@Config
public class FeedForwardTuner extends SubsystemBase {

    public static double frs = 0.08, fls = 0.07, brs = 0.07, bls = 0.08;
    public static double frv = 1, flv = 1, brv = 0.6, blv = 0.6;

    FeedForwardMotor frontRight, frontLeft, backLeft, backRight;
    Logger logger;

    public FeedForwardTuner(Logger logger_, FeedForwardMotor frontRight_, FeedForwardMotor frontLeft_, FeedForwardMotor backRight_, FeedForwardMotor backLeft_) {
        frontRight = frontRight_;
        frontLeft = frontLeft_;
        backRight = backRight_;
        backLeft = backLeft_;
        logger = logger_;

        frontRight.setFeedforwardCoefficients(frs, frv);
        frontLeft.setFeedforwardCoefficients(fls, flv);
        backRight.setFeedforwardCoefficients(brs, brv);
        backLeft.setFeedforwardCoefficients(bls, blv);
    }

    @Override
    public void periodic() {
        frontRight.setFeedforwardCoefficients(frs, frv);
        frontLeft.setFeedforwardCoefficients(fls, flv);
        backRight.setFeedforwardCoefficients(brs, brv);
        backLeft.setFeedforwardCoefficients(bls, blv);
    }
}
