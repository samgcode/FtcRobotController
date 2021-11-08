package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.DriveToPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.FindLevelCommand;
import org.firstinspires.ftc.teamcode.Commands.WaitForSecondsCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class Auto extends SequentialCommandGroup {
    SubsystemLocator subsystemLocator;

    double t = 23.75;//1 tile in inches
    Vector origin, homeA, homeB, hub, carousel, barrier, startingLocation, inWarehouse;
    int m = 1;

    double acceptableErrorXY = 1;
    double acceptableErrorH = 1;

    public Auto(boolean isRed, HardwareMap hardwareMap, Telemetry telemetry) {
        if(!isRed) {
            m = -1;
        }

        origin = new Vector(0, 0, 0);
//        homeA = new Vector(3*t*m, -1.5*t, 0);
//        homeB = new Vector(3*t*m, 0.5*t, 0);
        hub = new Vector(1*t, 1*t, 0);
        carousel = new Vector(1, -1*t, 0);
        barrier = new Vector(1, 2*t, 0);
        inWarehouse = new Vector(1, 3.5*t, 0);
        startingLocation = new Vector(0, 0, 0);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Logger logger = new Logger(dashboard);

        MecanumDrive driveSubsystem = SubsystemService.createMechanumDriveSubsystem(logger, hardwareMap,
                "drive3", "drive1", "drive2", "drive0");
        OdometrySubsystem odometrySubsystem = SubsystemService.createOdometrySubsystem(hardwareMap,
                "drive2", "drive3", "drive0", startingLocation);
        VisionSubsystem visionSubsystem = new VisionSubsystem(logger, hardwareMap);
        ContinuousServoSubsystem elevatorSubsystem = new ContinuousServoSubsystem(logger, hardwareMap, "servo0", "limit0", "limit4");
        new LogPosition(odometrySubsystem, logger);

        subsystemLocator = new SubsystemLocator(
                driveSubsystem, odometrySubsystem, visionSubsystem, dashboard, logger, hardwareMap, elevatorSubsystem
        );

//        SequentialCommandGroup autoCommand = new SequentialCommandGroup(
//                new SequentialCommandGroup(//score freight
//                        new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, hub, acceptableErrorXY, acceptableErrorH),
//                        new SequentialCommandGroup(//place on correct level
//                                new SetElevatorPositionVisionCommand(visionSubsystem, elevatorSubsystem, 1),
//                                new SetMotorPositionCommand(linearSubsystem, 10, 1)
//                                //deposit freight
//                        )
//                ),
//                new SequentialCommandGroup(//score carousel
//                        new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, carousel, acceptableErrorXY, acceptableErrorH)
//                        //turn carousel
//                ),
//                new SequentialCommandGroup(//park in warehouse
//                        new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, barrier, acceptableErrorXY, acceptableErrorH)
//                        //drive straight for 5 sec
//                )
//        );

        SequentialCommandGroup autoCommand = new SequentialCommandGroup(
                new FindLevelCommand(subsystemLocator, 3),
                new WaitForSecondsCommand(subsystemLocator, 1),
                new DriveToPositionCommand(subsystemLocator, hub, acceptableErrorXY, acceptableErrorH),
                new WaitForSecondsCommand(subsystemLocator,5),
                new DriveToPositionCommand(subsystemLocator, carousel, acceptableErrorXY, acceptableErrorH),
                new WaitForSecondsCommand(subsystemLocator,5),
                new DriveToPositionCommand(subsystemLocator, barrier, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, inWarehouse, acceptableErrorXY, acceptableErrorH)
        );

        addCommands(autoCommand);
//        addCommands(new SetElevatorPositionCommand(logger, hardwareMap, elevatorSubsystem, 1));
    }
}

/*
auto
>>score preloaded freight

>>>>find correct level
>>>>>>point toward barcode?
>>>>>>find shipping element position

>>>>put freight on correct level (vision subsystem)
>>>>>>move to hub
>>>>>>place freight on level X
>>>>>>>>lift arm to level X
>>>>>>>>deposit freight
>>>>>>>>lower arm

>>score carousel
>>>>move to carousel
>>>>turn carousel

>>park in warehouse
>>>>move to barrier
>>>>drive straight for 5 sec
*/