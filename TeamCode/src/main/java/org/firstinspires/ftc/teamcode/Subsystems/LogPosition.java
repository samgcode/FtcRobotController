package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import java.util.Locale;

public class LogPosition extends SubsystemBase {
    OdometrySubsystem odometrySubsystem;
    int frameCount = 0;

    public LogPosition(OdometrySubsystem odometrySubsystem_) {
        odometrySubsystem = odometrySubsystem_;
    }


    @Override
    public void periodic() {
        Pose2d pose = odometrySubsystem.getPose();
        String x = String.format(Locale.getDefault(), "%.3g", pose.getX());
        String y = String.format(Locale.getDefault(),"%.3g", pose.getY());
        String h = String.format(Locale.getDefault(),"%.3g", pose.getHeading() * (180/Math.PI));
        if(frameCount % 15 == 0) {
            System.out.println("Position: " + x + ", " + y + ", " + h);
        }
        frameCount++;
    }
}
