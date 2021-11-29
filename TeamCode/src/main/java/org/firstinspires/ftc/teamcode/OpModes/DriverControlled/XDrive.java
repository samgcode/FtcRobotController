package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.*;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Disabled
@TeleOp(name="XDrive", group="Drive")
public class XDrive extends CommandOpMode {
    DriveSubsystem driveSubsystem;
    CustomOdometrySubsystem odometrySubsystem;
    OdometrySubsystem ftcLibOdometry;

    SetDriveSpeedCommand setSpeedCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        odometrySubsystem = new CustomOdometrySubsystem(hardwareMap);
//        ftcLibOdometry = SubsystemService.createOdometrySubsystem(hardwareMap,
//                "motor2", "motor3", "motor0", new Vector(0, 0, 0));
//        new LogPosition(ftcLibOdometry);

        setSpeedCommand = new SetDriveSpeedCommand(driveSubsystem, gamepad1, odometrySubsystem);

        schedule(setSpeedCommand);

        telemetry.addData("Status", "Initialized");
    }
}