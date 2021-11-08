package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class SubsystemLocator {

    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    VisionSubsystem visionSubsystem;
    FtcDashboard dashboard;
    Logger logger;
    HardwareMap hardwareMap;
    ContinuousServoSubsystem elevatorSubsystem;

    Gamepad gamepad1, gamepad2;

    public SubsystemLocator(MecanumDrive driveSubsystem_, OdometrySubsystem odometrySubsystem_, VisionSubsystem visionSubsystem_, FtcDashboard dashboard_, Logger logger_, HardwareMap hardwareMap_, ContinuousServoSubsystem elevatorSubsystem_) {
        driveSubsystem = driveSubsystem_;
        odometrySubsystem = odometrySubsystem_;
        visionSubsystem = visionSubsystem_;
        dashboard = dashboard_;
        logger = logger_;
        hardwareMap = hardwareMap_;
        elevatorSubsystem = elevatorSubsystem_;
    }

    public SubsystemLocator(MecanumDrive driveSubsystem_, OdometrySubsystem odometrySubsystem_, VisionSubsystem visionSubsystem_, FtcDashboard dashboard_, Logger logger_, HardwareMap hardwareMap_, ContinuousServoSubsystem elevatorSubsystem_, Gamepad gamepad1_, Gamepad gamepad2_) {
        driveSubsystem = driveSubsystem_;
        odometrySubsystem = odometrySubsystem_;
        visionSubsystem = visionSubsystem_;
        dashboard = dashboard_;
        logger = logger_;
        hardwareMap = hardwareMap_;
        elevatorSubsystem = elevatorSubsystem_;
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;
    }

    public MecanumDrive getDriveSubsystem() { return driveSubsystem; }

    public OdometrySubsystem getOdometrySubsystem() { return odometrySubsystem; }

    public VisionSubsystem getVisionSubsystem() { return visionSubsystem;}

    public FtcDashboard getDashboard() { return dashboard; }

    public Logger getLogger() { return logger; }

    public HardwareMap getHardwareMap() { return hardwareMap; }

    public ContinuousServoSubsystem getElevatorSubsystem() { return elevatorSubsystem; }

    public Gamepad getGamepad1() { return gamepad1; }

    public Gamepad getGamepad2() { return gamepad2; }
}
