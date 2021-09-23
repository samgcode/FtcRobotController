package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.CustomOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class SetMechanumSpeedCommand extends CommandBase {
    MecanumDrive driveSubsystem;
    CustomOdometrySubsystem odometrySubsystem;
    Gamepad gamepad;

    public SetMechanumSpeedCommand(MecanumDrive driveSubsystem_, CustomOdometrySubsystem odometrySubsystem_, Gamepad gamepad_) {
        driveSubsystem = driveSubsystem_;
        odometrySubsystem = odometrySubsystem_;
        gamepad = gamepad_;

        addRequirements(odometrySubsystem);
    }

    @Override
    public void execute() {
        double speedModifier = 0.75;
        double h = -odometrySubsystem.getPosition().h;

        Vector speed = new Vector(-gamepad.left_stick_x*speedModifier, gamepad.left_stick_y*speedModifier, -gamepad.right_stick_x*speedModifier);

        driveSubsystem.driveFieldCentric(speed.x, speed.y, speed.h, h, true);
    }
}
