package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemService {
    public static OdometrySubsystem createOdometrySubsystem(HardwareMap hardwareMap, String leftEncoderName, String rightEncoderName, String centerEncoderName) {
        double TRACK_WIDTH = 10.4;
        double TICKS_TO_INCHES = (Math.PI*3.54331)/1120;
        double CENTER_WHEEL_OFFSET = 5.5;

        MotorEx leftEncoder = new MotorEx(hardwareMap, leftEncoderName);
        MotorEx rightEncoder = new MotorEx(hardwareMap, rightEncoderName);
        MotorEx centerEncoder = new MotorEx(hardwareMap, centerEncoderName);

        HolonomicOdometry holonomicOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                () -> centerEncoder.getCurrentPosition() * TICKS_TO_INCHES,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );
        return new OdometrySubsystem(holonomicOdometry);
    }

    public static MecanumDrive createMechanumDriveSubsystem(HardwareMap hardwareMap, String frontRightName, String frontLeftName, String backLeftName,String backRightName) {
        Motor frontRight, frontLeft, backLeft, backRight;
        frontRight = new Motor(hardwareMap, frontRightName);
        frontLeft = new Motor(hardwareMap, frontLeftName);
        backRight = new Motor(hardwareMap, backRightName);
        backLeft = new Motor(hardwareMap, backLeftName);

        return new MecanumDrive(frontLeft, frontRight, backLeft, backRight);
    }
}
