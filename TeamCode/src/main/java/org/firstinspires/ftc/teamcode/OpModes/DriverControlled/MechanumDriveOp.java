package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.SetMechanumSpeedCommand;
import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@TeleOp(name="Mechaunum drive", group="drive")
public class MechanumDriveOp extends CommandOpMode {
    MecanumDrive driveSubsystem;
    SetMechanumSpeedCommand setMechanumSpeedCommand;
    FtcDashboard dashboard;
    Logger logger;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        logger = new Logger(dashboard);

        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(logger, hardwareMap,
                "drive3", "drive1", "drive2", "drive0");
        OdometrySubsystem ftcLibOdometry = SubsystemService.createOdometrySubsystem(hardwareMap,
                "drive2", "drive3", "drive0", new Vector(0, 0, 0));
        new LogPosition(ftcLibOdometry, logger);

        setMechanumSpeedCommand = new SetMechanumSpeedCommand(logger, driveSubsystem, ftcLibOdometry, gamepad1);

        schedule(setMechanumSpeedCommand);
    }

}
