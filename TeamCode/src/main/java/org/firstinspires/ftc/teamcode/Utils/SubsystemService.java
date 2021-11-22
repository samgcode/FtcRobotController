package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SubsystemService {

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
