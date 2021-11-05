package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class ServiceLocator {

    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    VisionSubsystem visionSubsystem;
    FtcDashboard dashboard;
    Logger logger;
    ContinuousServoSubsystem elevatorSubsystem;

    public ServiceLocator(MecanumDrive driveSubsystem_, OdometrySubsystem odometrySubsystem_, VisionSubsystem visionSubsystem_, FtcDashboard dashboard_,  Logger logger_, ContinuousServoSubsystem elevatorSubsystem_) {
        driveSubsystem = driveSubsystem_;
        odometrySubsystem = odometrySubsystem_;
        visionSubsystem = visionSubsystem_;
        dashboard = dashboard_;
        logger = logger_;
        elevatorSubsystem = elevatorSubsystem_;
    }

    public MecanumDrive getDriveSubsystem() {
        return driveSubsystem;
    }

    public OdometrySubsystem getOdometrySubsystem() {
        return odometrySubsystem;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public FtcDashboard getDashboard() {
        return dashboard;
    }

    public Logger getLogger() {
        return logger;
    }

    public ContinuousServoSubsystem getElevatorSubsystem() {
        return elevatorSubsystem;
    }
}
