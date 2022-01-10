package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.CustomServo;
import org.firstinspires.ftc.teamcode.Subsystems.DriveElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class SubsystemLocator {

    MecanumDrive driveSubsystem;
    HolonomicOdometry holonomicOdometry;
    Odometry odometrySubsystem;
    VisionSubsystem visionSubsystem;
    FtcDashboard dashboard;
    Logger logger;
    HardwareMap hardwareMap;
    ElevatorSubsystem elevatorSubsystem;
    DriveElevatorSubsystem driveElevatorSubsystem;
    IntakeSubsystem intakeSubsystem;

    //hardware
    ContinuousServoSubsystem elevatorServo;
    CustomServo carouselServo;
    CustomServo intakeServo;
    ServoEx bucketServo;
    TouchSensor[] limitSwitches;
    Gamepad gamepad1, gamepad2;
    MotorEx leftEncoder, rightEncoder, centerEncoder;

    //constatnts
    String fr = "drive3", fl = "drive1", br = "drive0", bl = "drive2";
    double TRACK_WIDTH = 11.7;//13.7272565099261
    double WHEEL_DIAMETER = 1.365;
    double CENTER_WHEEL_OFFSET = -4.3;//
    // -6.79087916353029
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
        odometrySubsystem = new Odometry(holonomicOdometry, logger);
        odometrySubsystem.start();

        visionSubsystem = new VisionSubsystem(logger, hardwareMap);
        elevatorServo = new ContinuousServoSubsystem(logger, hardwareMap, "servo0", "limit1", "limit3");

        carouselServo = new CustomServo(logger, hardwareMap, "servo1");
        bucketServo = new SimpleServo(hardwareMap, "servo2", 0, 180);
        bucketServo.setInverted(false);
        intakeServo = new CustomServo(logger, hardwareMap_, "servo3");

        limitSwitches = new TouchSensor[]{
                hardwareMap.get(TouchSensor.class, "limit1"),
                hardwareMap.get(TouchSensor.class, "limit0"),
                hardwareMap.get(TouchSensor.class, "limit2"),
                hardwareMap.get(TouchSensor.class, "limit3"),
        };


        intakeSubsystem = new IntakeSubsystem(logger, intakeServo);
        elevatorSubsystem = new ElevatorSubsystem(logger, elevatorServo, bucketServo, limitSwitches);


//        new VoltagePrintOutSubsystem(logger, hardwareMap);
    }

    public SubsystemLocator(Vector startingLocation, HardwareMap hardwareMap_, Gamepad gamepad1_, Gamepad gamepad2_) {
        this(startingLocation, hardwareMap_);
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;
        driveElevatorSubsystem = new DriveElevatorSubsystem(logger, gamepad2, elevatorSubsystem);
    }

    public MecanumDrive getDriveSubsystem() { return driveSubsystem; }

    public Odometry getOdometrySubsystem() { return odometrySubsystem; }

    public VisionSubsystem getVisionSubsystem() { return visionSubsystem;}

    public FtcDashboard getDashboard() { return dashboard; }

    public Logger getLogger() { return logger; }

    public HardwareMap getHardwareMap() { return hardwareMap; }

    public ContinuousServoSubsystem getElevatorServo() { return elevatorServo; }

    public CustomServo getCarouselServo() { return carouselServo; }

    public ElevatorSubsystem getElevatorSubsystem() { return elevatorSubsystem; }

    public Gamepad getGamepad1() { return gamepad1; }

    public Gamepad getGamepad2() { return gamepad2; }

    public HolonomicOdometry getHolonomicOdometry() { return holonomicOdometry; }

    public IntakeSubsystem getIntakeSubsystem() { return intakeSubsystem; }

    public void resetEncoderOffsets(Vector pos) {
        leftEncoderOffset = leftEncoder.getCurrentPosition() * -TICKS_TO_INCHES - pos.x;
        rightEncoderOffset = rightEncoder.getCurrentPosition() * TICKS_TO_INCHES - pos.y;
        centerEncoderOffset = centerEncoder.getCurrentPosition() * -TICKS_TO_INCHES - pos.h;
    }

    public void stop() {
        logger.stop();
        odometrySubsystem.stop();
    }
}
