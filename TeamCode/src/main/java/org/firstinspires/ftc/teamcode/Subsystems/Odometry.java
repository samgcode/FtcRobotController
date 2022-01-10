package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.util.Locale;

@Config
public class Odometry implements Runnable {
    Thread thread;
    HolonomicOdometry odometry;
    Logger logger;
    int frameCount;

    public static int logTime = 100;

    public Odometry(HolonomicOdometry odometry_, Logger logger_) {
        odometry = odometry_;
        logger = logger_;
    }

    public void start() {
        thread = new Thread(this, "odometry");
        thread.start();
    }

    @Override
    public void run() {
        while(!thread.isInterrupted()) {
            updatePose();
            if(frameCount % logTime == 0) {
                Pose2d pose = getPose();
                String x = String.format(Locale.getDefault(),"%.3g", pose.getX());
                String y = String.format(Locale.getDefault(),"%.3g", pose.getY());
                String h = String.format(Locale.getDefault(),"%.3g", pose.getHeading() * (180/Math.PI));
                logger.log("X", x);
                logger.log("y", y);
                logger.log("h", h);
            }
            frameCount++;
        }
    }

    public synchronized void updatePose() {
        odometry.updatePose();
    }

    public synchronized void updatePose(Pose2d pose) {
        odometry.updatePose(pose);
    }

    public synchronized Pose2d getPose() {
        System.out.println("get");
        return odometry.getPose();
    }

    public void stop() {
        thread.interrupt();
    }
}
