package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.SetMechanumSpeedCommand;
import org.firstinspires.ftc.teamcode.Subsystems.CustomOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;

@TeleOp(name="Mechaunum drive", group="drive")
public class MechanumDriveOp extends CommandOpMode {
    MecanumDrive driveSubsystem;
    CustomOdometrySubsystem odometrySubsystem;
    SetMechanumSpeedCommand setMechanumSpeedCommand;

    @Override
    public void initialize() {
        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(hardwareMap,
                "motor3", "motor1", "motor2", "motor0");
        odometrySubsystem = new CustomOdometrySubsystem(hardwareMap);

        setMechanumSpeedCommand = new SetMechanumSpeedCommand(driveSubsystem, odometrySubsystem, gamepad1);

        schedule(setMechanumSpeedCommand);
    }

}
