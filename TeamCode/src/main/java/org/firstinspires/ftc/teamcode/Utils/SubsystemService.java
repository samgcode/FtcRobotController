package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemService {
    public static OdometrySubsystem createOdometrySubsystem(HardwareMap hardwareMap, String leftEncoderName, String rightEncoderName, String centerEncoderName, Vector initialPosition) {
        double TRACK_WIDTH = 11.8;//13.7272565099261
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

    public static MecanumDrive createMechanumDriveSubsystem(Logger logger, HardwareMap hardwareMap, String frontRightName, String frontLeftName, String backLeftName,String backRightName) {
        FeedForwardMotor frontRight, frontLeft, backLeft, backRight;
        frontRight = new FeedForwardMotor(hardwareMap, frontRightName);
        frontLeft = new FeedForwardMotor(hardwareMap, frontLeftName);
        backRight = new FeedForwardMotor(hardwareMap, backRightName);
        backLeft = new FeedForwardMotor(hardwareMap, backLeftName);

        new FeedForwardTuner(logger, frontRight, frontLeft, backRight, backLeft);

        return new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }

    public static MecanumDrive createMechanumDriveSubsystem(FeedForwardMotor frontRight, FeedForwardMotor frontLeft, FeedForwardMotor backRight, FeedForwardMotor backLeft) {
        return new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }
}
