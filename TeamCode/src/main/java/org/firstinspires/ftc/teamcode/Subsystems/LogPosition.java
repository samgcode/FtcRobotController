package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.util.Locale;

public class LogPosition extends SubsystemBase {
    OdometrySubsystem odometrySubsystem;
    int frameCount = 0;
    Logger logger;

    public LogPosition(OdometrySubsystem odometrySubsystem_) {
        odometrySubsystem = odometrySubsystem_;
//        logger = logger_;
    }
    public LogPosition(OdometrySubsystem odometrySubsystem_, Logger logger_) {
        odometrySubsystem = odometrySubsystem_;
        logger = logger_;
    }


    @Override
    public void periodic() {
        Pose2d pose = odometrySubsystem.getPose();
        String x = String.format(Locale.getDefault(), "%.3g", pose.getX());
        String y = String.format(Locale.getDefault(),"%.3g", pose.getY());
        String h = String.format(Locale.getDefault(),"%.3g", pose.getHeading() * (180/Math.PI));
        if(frameCount % 15 == 0) {
            logger.log("X", x);
            logger.log("y", y);
            logger.log("h", h);
        }
        frameCount++;
    }
}
