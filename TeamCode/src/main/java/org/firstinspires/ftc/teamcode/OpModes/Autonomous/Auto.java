package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Commands.DriveToPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.FindLevelCommand;
import org.firstinspires.ftc.teamcode.Commands.RotateBucketCommand;
import org.firstinspires.ftc.teamcode.Commands.RotateCarouselCommand;
import org.firstinspires.ftc.teamcode.Commands.SetCurrentPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.SetElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.WaitForSecondsCommand;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class Auto extends SequentialCommandGroup {
    SubsystemLocator subsystemLocator;

    double t = 23.75;//1 tile in inches
    Vector origin, tempA, tempB, tempBR, tempC, hub, carousel, barrier, startingLocation, inWarehouse;
    int m = 1;

    double acceptableErrorXY = 0.5;
    double acceptableErrorH = 1;

    public Auto(boolean isRed, SubsystemLocator subsystemLocator) {
        if(!isRed) {
            m = -1;
        }

        origin = new Vector(0, 0, 0);
        hub = new Vector(1*t, 1*t, 0);
        carousel = new Vector(9, -1*(t+1), -90);
        barrier = new Vector(0, 2*t, 0);
        inWarehouse = new Vector(0, 3.5*t, 0);
        startingLocation = new Vector(0, 0, 0);
        tempA = new Vector(0, 1*t, 0);
        tempB = new Vector(10, -0.5*t, 0);
        tempBR = new Vector(10, -0.5*t, -90);
        tempC = new Vector(-1, 0, 0);

        SequentialCommandGroup autoCommand = new SequentialCommandGroup(
                new SetCurrentPositionCommand(subsystemLocator, new Vector(0, 0, 0)),
                new SequentialCommandGroup( //find level
                        new FindLevelCommand(subsystemLocator, 3)
                ),
                new SequentialCommandGroup( //place on hub
                        new WaitForSecondsCommand(subsystemLocator, 1),
                        new DriveToPositionCommand(subsystemLocator, tempA, acceptableErrorXY, acceptableErrorH),
                        new WaitForSecondsCommand(subsystemLocator, 1),
                        new DriveToPositionCommand(subsystemLocator, hub, acceptableErrorXY, acceptableErrorH),
                        new SetElevatorPositionCommand(subsystemLocator, true),
                        new WaitForSecondsCommand(subsystemLocator, 1),
                        new DriveToPositionCommand(subsystemLocator, tempA, acceptableErrorXY, acceptableErrorH)
                )
//                new SequentialCommandGroup( //carousel
//                        new WaitForSecondsCommand(subsystemLocator,1),
//                        new DriveToPositionCommand(subsystemLocator, tempB, acceptableErrorXY, acceptableErrorH),
//                        new DriveToPositionCommand(subsystemLocator, tempBR, acceptableErrorXY, acceptableErrorH),
//                        new DriveToPositionCommand(subsystemLocator, carousel, acceptableErrorXY, acceptableErrorH),
//                        new RotateCarouselCommand(subsystemLocator),
//                        new DriveToPositionCommand(subsystemLocator, tempBR, acceptableErrorXY, acceptableErrorH),
//                        new DriveToPositionCommand(subsystemLocator, tempB, acceptableErrorXY, acceptableErrorH)
//                ),
//                new SequentialCommandGroup( //reset position
//                        new DriveToPositionCommand(subsystemLocator, new Vector(-2, 0, 0), 2, acceptableErrorH, 4),
//                        new SetCurrentPositionCommand(subsystemLocator, startingLocation)
//                ),
//                new SequentialCommandGroup( //park in warehouse
//                        new WaitForSecondsCommand(subsystemLocator,3),
//                        new DriveToPositionCommand(subsystemLocator, barrier, acceptableErrorXY, acceptableErrorH),
//                        new DriveToPositionCommand(subsystemLocator, inWarehouse, acceptableErrorXY, acceptableErrorH)
//                ),
//                new WaitForSecondsCommand(subsystemLocator, 1)
        );

//        addCommands(autoCommand);
        addCommands(
                new SequentialCommandGroup(
                        new SetElevatorPositionCommand(subsystemLocator, 1),
                        new RotateBucketCommand(subsystemLocator, 90),
                        new RotateBucketCommand(subsystemLocator, 180),
                        new RotateBucketCommand(subsystemLocator, 90)
                )
        );
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