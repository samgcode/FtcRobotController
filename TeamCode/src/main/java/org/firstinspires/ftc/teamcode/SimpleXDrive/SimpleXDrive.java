package org.firstinspires.ftc.teamcode.SimpleXDrive;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Subsystems.*;
import org.firstinspires.ftc.teamcode.Commands.*;


@Disabled
@TeleOp(name="SimpleXDrive", group="Drive")
public class SimpleXDrive extends CommandOpMode {
    public DcMotor[] xMotors;
    public DcMotor[] yMotors;

    DriveSubsystem driveSubsystem;
    CustomOdometrySubsystem odometrySubsystem;

    SetDriveSpeedCommand setSpeedCommand;

    @Override
    public void initialize() {
        driveSubsystem = new DriveSubsystem(hardwareMap);
        odometrySubsystem = new CustomOdometrySubsystem(hardwareMap, telemetry);

        setSpeedCommand = new SetDriveSpeedCommand(driveSubsystem, gamepad1, odometrySubsystem);

        schedule(setSpeedCommand);

        telemetry.addData("Status", "Initialized");

    }
}
