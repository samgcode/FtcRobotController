package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class SetMechanumSpeedCommand extends CommandBase {
    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    GamepadEx gamepad;
    PIDController hPID;
    Logger logger;
    double targetAngle;
    boolean updateTargetAngle;
    double speedModifier = 1.0;

    public static PIDCoefficients hPidCoefficients = new PIDCoefficients(0.04,1,0.015);
    public static double fastSpeed = 1;
    public static double slowSpeed = 0.5;

    public SetMechanumSpeedCommand(SubsystemLocator subsystemLocator) {
        driveSubsystem = subsystemLocator.getDriveSubsystem();
        odometrySubsystem = subsystemLocator.getOdometrySubsystem();
        gamepad = new GamepadEx(subsystemLocator.getGamepad1());
        logger = subsystemLocator.getLogger();

        hPID = new PIDController(0,0,0);

        addRequirements(odometrySubsystem);
    }

    boolean prevState = false;

    @Override
    public void execute() {
        double hSpeedModifier = 0.5;
        Vector pos = new Vector(odometrySubsystem.getPose());
        double h = pos.h;

        logger.log("target angle", targetAngle);

        double measuredAngle = Angle.getMAngle(h, targetAngle);

        hPID.setPID(hPidCoefficients.p, hPidCoefficients.i, hPidCoefficients.d);
        hPID.setTolerance(1);
        hPID.setSetPoint(targetAngle);

        double hSpeed = hPID.calculate(measuredAngle);
        Range.clip(hSpeed, -hSpeedModifier, hSpeedModifier);

        boolean state = gamepad.isDown(GamepadKeys.Button.A);
        if(prevState != state) {
            if(state) {
                speedModifier = speedModifier == slowSpeed ? fastSpeed : slowSpeed;
            }
            prevState = state;
        }

        if(gamepad.getRightX() > 0.2 || gamepad.getRightX() < -0.2) {
            hSpeed = gamepad.getRightX()*speedModifier;
            updateTargetAngle = true;
            logger.log("status", "rotating");
        } else {
            if(updateTargetAngle) {
                targetAngle = h;
                updateTargetAngle = false;
            }
            logger.log("status", "pid");
        }

        Vector speed = new Vector(-gamepad.getLeftX()*speedModifier, gamepad.getLeftY()*speedModifier, hSpeed);

       driveSubsystem.driveFieldCentric(speed.x, speed.y, speed.h, h, true);
        //driveSubsystem.driveRobotCentric(speed.x, speed.y, speed.h, true);
    }
}
