package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.DriveToPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.FindLevelCommand;
import org.firstinspires.ftc.teamcode.Commands.SetCurrentPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.WaitForSecondsCommand;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class Auto extends SequentialCommandGroup {
    SubsystemLocator subsystemLocator;

    double t = 23.75;//1 tile in inches
    Vector origin, tempA, tempB, tempC, hub, carousel, barrier, startingLocation, inWarehouse;
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
        carousel = new Vector(10, -1*(t+2), -90);
        barrier = new Vector(1, 2*t, 0);
        inWarehouse = new Vector(1, 3.5*t, 0);
        startingLocation = new Vector(0, 0, 0);
        tempA = new Vector(1, 1*t, 0);
        tempB = new Vector(10, -0.5*t, 0);
        tempC = new Vector(-1, 0, 0);

        subsystemLocator = new SubsystemLocator(startingLocation, hardwareMap);

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
                new SetCurrentPositionCommand(subsystemLocator, new Vector(0, 0, 0)),
                new FindLevelCommand(subsystemLocator, 3),
                new WaitForSecondsCommand(subsystemLocator, 1),
                new DriveToPositionCommand(subsystemLocator, tempA, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, hub, acceptableErrorXY, acceptableErrorH),
//                new SetElevatorPositionCommand(subsystemLocator, true),
                new WaitForSecondsCommand(subsystemLocator,1),
                new DriveToPositionCommand(subsystemLocator, tempA, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, carousel, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, tempB, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(-2, 0, 0), 2, acceptableErrorH),
                new SetCurrentPositionCommand(subsystemLocator, startingLocation),
                new WaitForSecondsCommand(subsystemLocator,3),
                new DriveToPositionCommand(subsystemLocator, barrier, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, inWarehouse, acceptableErrorXY, acceptableErrorH),
                new SetCurrentPositionCommand(subsystemLocator, startingLocation)
        );

        addCommands(autoCommand);
//        addCommands(
//                new SequentialCommandGroup(
//                        new SetCurrentPositionCommand(subsystemLocator, new Vector(10,10,0)),
//                        new DriveToPositionCommand(subsystemLocator, new Vector(20, 20, 0), acceptableErrorXY, acceptableErrorH)
//                )
//        );
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