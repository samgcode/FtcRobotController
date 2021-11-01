package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Commands.DriveToPositionCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Utils.FeedForwardMotor;
import org.firstinspires.ftc.teamcode.Utils.FeedForwardTuner;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Autonomous
public class PurePursuitTesting extends CommandOpMode {
    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    LogPosition logPosition;
    Logger logger;
    FeedForwardTuner feedForwardTuner;

    double t = 23.75;//1 tile in inches
    Vector origin, homeA, homeB, hub, carousel, barrier, startingLocation;
    int m = 1;

    double acceptableErrorXY = 1;
    double acceptableErrorH = 2;

    @Override
    public void initialize() {
        origin = new Vector(0, 0, 0);
        homeA = new Vector(3*t*m, -1.5*t, -90*m);
        homeB = new Vector(3*t*m, 0.5*t, -90*m);
        hub = new Vector(1.5*t*m, -0.5*t, -90*m);
        carousel = new Vector(2.5*t*m, -2.5*t, -90*m);
        barrier = new Vector(2*t*m, 0.5*t, 0);
        startingLocation = new Vector(3*t*m, -0.5*t, -90*m);

        logger = new Logger(FtcDashboard.getInstance());


        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(logger, hardwareMap,
                "drive3", "drive1", "drive2", "drive0");
        odometrySubsystem = SubsystemService.createOdometrySubsystem(hardwareMap,
                "drive2", "drive3", "drive0", origin);

        logPosition = new LogPosition(odometrySubsystem, logger);

        Vector pointA = new Vector(20, 0, 90);
        Vector pointB = new Vector(10, 10, -90);
        Vector pointC = new Vector(0, 15, -180);
        Vector pointD = new Vector(10, 10, 180);

        SequentialCommandGroup movementCommand = new SequentialCommandGroup(
                new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, origin, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, pointA, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, pointB, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, pointC, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, pointD, acceptableErrorXY, acceptableErrorH),
                new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, origin, acceptableErrorXY, acceptableErrorH)
        );

//        schedule(movementCommand);

//        schedule(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, origin, acceptableErrorXY, acceptableErrorH));

        setupGamepads();
    }

    void setupGamepads() {
        GamepadEx gamepadEx = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        Button a = new GamepadButton(gamepadEx, GamepadKeys.Button.A);
        Button b = new GamepadButton(gamepadEx, GamepadKeys.Button.B);
        Button y = new GamepadButton(gamepadEx, GamepadKeys.Button.Y);
        Button x = new GamepadButton(gamepadEx, GamepadKeys.Button.X);

        Button a2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.A);
        Button b2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.B);
        Button y2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.Y);
        Button x2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.X);

        a.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(0, 0, 0), acceptableErrorXY, acceptableErrorH));
        b.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(10, 10, 0), acceptableErrorXY, acceptableErrorH));
        y.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(20, 0, 0), acceptableErrorXY, acceptableErrorH));
        x.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(10, -10, 0), acceptableErrorXY, acceptableErrorH));

        a2.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(0, 0, 0), acceptableErrorXY, acceptableErrorH));
        b2.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(0, 0, 90), acceptableErrorXY, acceptableErrorH));
        y2.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(0, 0, 180), acceptableErrorXY, acceptableErrorH));
        x2.whenPressed(new DriveToPositionCommand(logger, driveSubsystem, odometrySubsystem, new Vector(0, 0, -90), acceptableErrorXY, acceptableErrorH));
    }

}
