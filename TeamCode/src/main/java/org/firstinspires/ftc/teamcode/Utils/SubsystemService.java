package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
util functions for creating subsystems
 */
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
}
