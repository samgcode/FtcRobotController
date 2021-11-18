package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.TreeMap;
import java.util.concurrent.ArrayBlockingQueue;

import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class SubsystemLocator {

    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    VisionSubsystem visionSubsystem;
    FtcDashboard dashboard;
    Logger logger;
    HardwareMap hardwareMap;
    ElevatorSubsystem elevatorSubsystem;

    //hardware
    ContinuousServoSubsystem elevatorServo;
    TouchSensor[] limitSwitches;
    Gamepad gamepad1, gamepad2;

    public SubsystemLocator(MecanumDrive driveSubsystem_, OdometrySubsystem odometrySubsystem_, VisionSubsystem visionSubsystem_, FtcDashboard dashboard_, Logger logger_, HardwareMap hardwareMap_, ContinuousServoSubsystem elevatorSubsystem_, Gamepad gamepad1_, Gamepad gamepad2_) {
        driveSubsystem = driveSubsystem_;
        odometrySubsystem = odometrySubsystem_;
        visionSubsystem = visionSubsystem_;
        dashboard = dashboard_;
        logger = logger_;
        hardwareMap = hardwareMap_;
        elevatorServo = elevatorSubsystem_;
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;
    }

    public SubsystemLocator(HardwareMap hardwareMap_, ElevatorSubsystem elevatorSubsystem_, ContinuousServoSubsystem elevatorServo_, Gamepad gamepad1_, Gamepad gamepad2_) {
        hardwareMap = hardwareMap_;
        elevatorServo = elevatorServo_;
        elevatorSubsystem = elevatorSubsystem_;
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;

        dashboard = FtcDashboard.getInstance();

        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(logger, hardwareMap,
                "drive3", "drive1", "drive2", "drive0");
        odometrySubsystem = SubsystemService.createOdometrySubsystem(hardwareMap,
                "drive2", "drive3", "drive0", new Vector(0,0,0));

        visionSubsystem = new VisionSubsystem(logger, hardwareMap);

        new LogPosition(odometrySubsystem, logger);

    }

    public SubsystemLocator(Vector startingLocation, HardwareMap hardwareMap_) {
        hardwareMap = hardwareMap_;

        dashboard = FtcDashboard.getInstance();
        logger = new Logger();
        logger.start();


        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(logger, hardwareMap,
                "drive3", "drive1", "drive2", "drive0");
        odometrySubsystem = SubsystemService.createOdometrySubsystem(hardwareMap,
                "drive2", "drive3", "drive0", startingLocation);
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
//        new LogPosition(odometrySubsystem, logger);

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

    public ElevatorSubsystem getElevatorServo() { return elevatorSubsystem; }

    public ElevatorSubsystem getElevatorSubsystem() { return elevatorSubsystem; }

    public Gamepad getGamepad1() { return gamepad1; }

    public Gamepad getGamepad2() { return gamepad2; }
}
