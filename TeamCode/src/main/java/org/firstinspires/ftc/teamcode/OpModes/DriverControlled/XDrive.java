package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.*;

@TeleOp(name="XDrive", group="Drive")
public class XDrive extends CommandOpMode {
    DriveSubsystem driveSubsystem;
    CustomOdometrySubsystem odometrySubsystem;

    SetDriveSpeedCommand setSpeedCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        odometrySubsystem = new CustomOdometrySubsystem(hardwareMap);

        setSpeedCommand = new SetDriveSpeedCommand(driveSubsystem, gamepad1, odometrySubsystem);

        schedule(setSpeedCommand);

        telemetry.addData("Status", "Initialized");
    }
}