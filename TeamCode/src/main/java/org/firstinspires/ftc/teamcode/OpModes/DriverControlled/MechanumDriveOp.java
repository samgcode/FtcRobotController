package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.SetMechanumSpeedCommand;
import org.firstinspires.ftc.teamcode.Subsystems.CustomOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@TeleOp(name="Mechaunum drive", group="drive")
public class MechanumDriveOp extends CommandOpMode {
    MecanumDrive driveSubsystem;
    //CustomOdometrySubsystem odometrySubsystem;
    SetMechanumSpeedCommand setMechanumSpeedCommand;

    @Override
    public void initialize() {
        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(hardwareMap,
                "drive3", "drive1", "drive2", "drive0");
        //odometrySubsystem = new CustomOdometrySubsystem(hardwareMap);
        OdometrySubsystem ftcLibOdometry = SubsystemService.createOdometrySubsystem(hardwareMap,
                "drive2", "drive3", "drive0", new Vector(0, 0, 0));
        new LogPosition(ftcLibOdometry);

        setMechanumSpeedCommand = new SetMechanumSpeedCommand(driveSubsystem, ftcLibOdometry, gamepad1);

        schedule(setMechanumSpeedCommand);
    }

}
