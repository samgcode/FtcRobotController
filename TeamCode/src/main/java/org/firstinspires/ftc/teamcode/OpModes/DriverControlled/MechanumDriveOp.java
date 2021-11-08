package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.SetMechanumSpeedCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@TeleOp(name="Mechaunum drive", group="drive")
public class MechanumDriveOp extends CommandOpMode {
    SubsystemLocator subsystemLocator;

    @Override
    public void initialize() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Logger logger = new Logger(dashboard);

        MecanumDrive driveSubsystem = SubsystemService.createMechanumDriveSubsystem(logger, hardwareMap,
                "drive3", "drive1", "drive2", "drive0");
        OdometrySubsystem odometrySubsystem = SubsystemService.createOdometrySubsystem(hardwareMap,
                "drive2", "drive3", "drive0", new Vector(0,0,0));
        VisionSubsystem visionSubsystem = new VisionSubsystem(logger, hardwareMap);
        ContinuousServoSubsystem elevatorSubsystem = new ContinuousServoSubsystem(logger, hardwareMap, "servo0", "limit0", "limit4");
        new LogPosition(odometrySubsystem, logger);

        subsystemLocator = new SubsystemLocator(
                driveSubsystem, odometrySubsystem, visionSubsystem, dashboard, logger, hardwareMap, elevatorSubsystem, gamepad1, gamepad2
        );

        schedule(new SetMechanumSpeedCommand(subsystemLocator));
    }

}
