package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class SubsystemLocator {

    MecanumDrive driveSubsystem;
    HolonomicOdometry holonomicOdometry;
    OdometrySubsystem odometrySubsystem;
    VisionSubsystem visionSubsystem;
    FtcDashboard dashboard;
    Logger logger;
    HardwareMap hardwareMap;
    ElevatorSubsystem elevatorSubsystem;

    //hardware
    ContinuousServoSubsystem elevatorServo;
    ContinuousServoSubsystem carouselServo;
    TouchSensor[] limitSwitches;
    Gamepad gamepad1, gamepad2;
    MotorEx leftEncoder, rightEncoder, centerEncoder;

    //constatnts
    String fr = "drive3", fl = "drive1", br = "drive0", bl = "drive2";
    double TRACK_WIDTH = 11.9;//13.7272565099261
    double WHEEL_DIAMETER = 1.366;
    double CENTER_WHEEL_OFFSET = -4.2;//-6.79087916353029
    double TICKS_PER_REV = 8192;//2048
    double TICKS_TO_INCHES = WHEEL_DIAMETER * Math.PI / TICKS_PER_REV;

    //
    double leftEncoderOffset = 0;
    double rightEncoderOffset = 0;
    double centerEncoderOffset = 0;

    public SubsystemLocator(Vector startingLocation, HardwareMap hardwareMap_) {
        hardwareMap = hardwareMap_;

        dashboard = FtcDashboard.getInstance();
        logger = new Logger();
        logger.start();


        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(logger, hardwareMap,fr, fl, bl, br);

        leftEncoder = new MotorEx(hardwareMap, bl);
        rightEncoder = new MotorEx(hardwareMap, fr);
        centerEncoder = new MotorEx(hardwareMap, br);

        holonomicOdometry = new HolonomicOdometry(
                () -> leftEncoder.getCurrentPosition() * -TICKS_TO_INCHES - leftEncoderOffset,
                () -> rightEncoder.getCurrentPosition() * TICKS_TO_INCHES - rightEncoderOffset,
                () -> centerEncoder.getCurrentPosition() * -TICKS_TO_INCHES - centerEncoderOffset,
                TRACK_WIDTH, CENTER_WHEEL_OFFSET
        );

        holonomicOdometry.updatePose(startingLocation.toPose2d());
        odometrySubsystem = new OdometrySubsystem(holonomicOdometry);

        visionSubsystem = new VisionSubsystem(logger, hardwareMap);
        elevatorServo = new ContinuousServoSubsystem(logger, hardwareMap, "servo0", "limit0", "limit4");

        limitSwitches = new TouchSensor[]{
                hardwareMap.get(TouchSensor.class, "limit0"),
                hardwareMap.get(TouchSensor.class, "limit1"),
                hardwareMap.get(TouchSensor.class, "limit2"),
                hardwareMap.get(TouchSensor.class, "limit3"),
                hardwareMap.get(TouchSensor.class, "limit4"),
        };

        elevatorSubsystem = new ElevatorSubsystem(logger, elevatorServo, limitSwitches);
        new LogPosition(odometrySubsystem, logger);

//        new VoltagePrintOutSubsystem(logger, hardwareMap);
    }

    public SubsystemLocator(Vector startingLocation, HardwareMap hardwareMap_, Gamepad gamepad1_, Gamepad gamepad2_) {
        this(startingLocation, hardwareMap_);
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;
    }

    public MecanumDrive getDriveSubsystem() { return driveSubsystem; }

    public OdometrySubsystem getOdometrySubsystem() { return odometrySubsystem; }

    public VisionSubsystem getVisionSubsystem() { return visionSubsystem;}

    public FtcDashboard getDashboard() { return dashboard; }

    public Logger getLogger() { return logger; }

    public HardwareMap getHardwareMap() { return hardwareMap; }

    public ContinuousServoSubsystem getElevatorServo() { return elevatorServo; }

    public ElevatorSubsystem getElevatorSubsystem() { return elevatorSubsystem; }

    public Gamepad getGamepad1() { return gamepad1; }

    public Gamepad getGamepad2() { return gamepad2; }

    public HolonomicOdometry getHolonomicOdometry() { return holonomicOdometry; }

    public void resetEncoderOffsets() {
        leftEncoderOffset = leftEncoder.getCurrentPosition() * -TICKS_TO_INCHES;
        rightEncoderOffset = rightEncoder.getCurrentPosition() * TICKS_TO_INCHES;
        centerEncoderOffset = centerEncoder.getCurrentPosition() * -TICKS_TO_INCHES;
    }
}
