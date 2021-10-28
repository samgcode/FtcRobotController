package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemService {
    public static OdometrySubsystem createOdometrySubsystem(HardwareMap hardwareMap, String leftEncoderName, String rightEncoderName, String centerEncoderName, Vector initialPosition) {
        double TRACK_WIDTH = 11.6715657;//13.7272565099261
        double WHEEL_DIAMETER = 1.366;
        double CENTER_WHEEL_OFFSET = -5;//-6.79087916353029
        double TICKS_PER_REV = 8192;//2048
        double TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;


        MotorEx leftEncoder = new MotorEx(hardwareMap, leftEncoderName);
        MotorEx rightEncoder = new MotorEx(hardwareMap, rightEncoderName);
        MotorEx centerEncoder = new MotorEx(hardwareMap, centerEncoderName);

//        rightEncoder.setInverted(false);

        HolonomicOdometry holonomicOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * -TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> centerEncoder.getCurrentPosition() * -TICKS_TO_INCHES,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        holonomicOdometry.updatePose(initialPosition.toPose2d());

        return new OdometrySubsystem(holonomicOdometry);
    }

    public static MecanumDrive createMechanumDriveSubsystem(HardwareMap hardwareMap, String frontRightName, String frontLeftName, String backLeftName,String backRightName) {
        Motor frontRight, frontLeft, backLeft, backRight;
        frontRight = new Motor(hardwareMap, frontRightName);
        frontLeft = new Motor(hardwareMap, frontLeftName);
        backRight = new Motor(hardwareMap, backRightName);
        backLeft = new Motor(hardwareMap, backLeftName);
//        frontLeft.setInverted(true);
//        frontRight.setInverted(true);
//        backLeft.setInverted(true);
//        backRight.setInverted(true);
        return new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }
}
