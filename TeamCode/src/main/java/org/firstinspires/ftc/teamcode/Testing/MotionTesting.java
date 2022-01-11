package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.DriveToPositionCommand;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

/*
opmode for testing motion related features
 */
@Config
@Autonomous
public class MotionTesting extends CommandOpMode {
    SubsystemLocator subsystemLocator;
    public static double acceptableErrorXY = 0.25;
    public static double acceptableErrorH = 1;

    @Override
    public void initialize() {
        subsystemLocator = new SubsystemLocator(new Vector(0,0,0), hardwareMap);

        SequentialCommandGroup positions = new SequentialCommandGroup(
                new DriveToPositionCommand(subsystemLocator, new Vector(0,0,0), acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(10,0,0), acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(10,10,0), acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(0,10,0), acceptableErrorXY, acceptableErrorH)
        );

//        positions = new SequentialCommandGroup(
//                new DriveToPositionCommand(subsystemLocator, new Vector(0,0,0), acceptableErrorXY, acceptableErrorH),
//                new DriveToPositionCommand(subsystemLocator, new Vector(0,0,180), acceptableErrorXY, acceptableErrorH),
//                new DriveToPositionCommand(subsystemLocator, new Vector(0,0,90), acceptableErrorXY, acceptableErrorH),
//                new DriveToPositionCommand(subsystemLocator, new Vector(0,0,-90), acceptableErrorXY, acceptableErrorH)
//        );

        positions = new SequentialCommandGroup(
                new DriveToPositionCommand(subsystemLocator, new Vector(0,0,0), acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(0,10,0), acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(0,10,90), acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(10,10,90), acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(subsystemLocator, new Vector(10,0,-90), acceptableErrorXY, acceptableErrorH)
        );

        SequentialCommandGroup loopPositions = new SequentialCommandGroup(
                positions,
                positions,
                positions,
                positions,
                positions,
                positions,
                positions,
                positions,
                positions,
                positions
        );

        schedule(new SequentialCommandGroup(
                loopPositions,
                loopPositions,
                loopPositions,
                loopPositions,
                loopPositions,
                loopPositions,
                loopPositions,
                loopPositions,
                loopPositions
        ));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        subsystemLocator.stop();
        reset();
    }
}
