package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Vector;

/*
a custom implementation of the FTC Lib odometry
mostly deprecated, use FTC Lib OdometrySubsystem instead
 */
public class CustomOdometrySubsystem extends SubsystemBase {
    MotorEx encoderLeft, encoderRight, encoderPerp;
    RevIMU imu;
    HolonomicOdometry holOdom;
    HardwareMap hardwareMap;

    static double TRACK_WIDTH = 10.4;
    static double TICKS_TO_INCHES = (Math.PI*3.54331)/1120;
    static double CENTER_WHEEL_OFFSET = -5.5;

    Vector position;

    public CustomOdometrySubsystem(HardwareMap hMap_) {
        hardwareMap = hMap_;

        encoderLeft = new MotorEx(hardwareMap, "motor2");
        encoderRight = new MotorEx(hardwareMap, "motor3");
        encoderPerp = new MotorEx(hardwareMap, "motor0");


        holOdom = new HolonomicOdometry(
                () -> encoderLeft.getCurrentPosition() * -TICKS_TO_INCHES,
                () -> encoderRight.getCurrentPosition() * TICKS_TO_INCHES,
                () -> encoderPerp.getCurrentPosition() * -TICKS_TO_INCHES,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        imu = new RevIMU(hardwareMap, "imu");

        holOdom.updatePose(new Pose2d(1, 2, new Rotation2d(0)));
        imu.init();
    }

    @Override
    public void periodic() {
        holOdom.updatePose();

        Pose2d pose = holOdom.getPose();
        double h = -imu.getHeading();

        position = new Vector(pose.getX(), pose.getY(), h);
    }

    public Vector getPosition() {
        return position;
    }
}
