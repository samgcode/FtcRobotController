package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

/*
command that updates the drive speed of the robot based on controller inputs
pid is for the heading to attempt to keep the robot from turning while translating sideways
 */
@Config
public class SetMechanumSpeedCommand extends CommandBase {
    MecanumDrive driveSubsystem;
    Odometry odometrySubsystem;
    GamepadEx gamepad;
    PIDController hPIDDrive;
    Logger logger;
    double targetAngle;
    boolean updateTargetAngle;
    double speedModifier = 1.0;

    public static PIDCoefficients hPidCoefficientsDrive = new PIDCoefficients(0.001, 0.2, 0.00018);
    public static double fastSpeed = 1;
    public static double slowSpeed = 0.5;

    public SetMechanumSpeedCommand(SubsystemLocator subsystemLocator) {
        driveSubsystem = subsystemLocator.getDriveSubsystem();
        odometrySubsystem = subsystemLocator.getOdometrySubsystem();
        gamepad = new GamepadEx(subsystemLocator.getGamepad1());
        logger = subsystemLocator.getLogger();

        hPIDDrive = new PIDController(0,0,0);
    }

    boolean prevState = false;

    @Override
    public void execute() {
        double hSpeedModifier = 0.5;
        Vector pos = new Vector(odometrySubsystem.getPose());
        double h = pos.h;

        double measuredAngle = Angle.getMAngle(h, targetAngle);

        hPIDDrive.setPID(hPidCoefficientsDrive.p, hPidCoefficientsDrive.i, hPidCoefficientsDrive.d);
        hPIDDrive.setTolerance(1);
        hPIDDrive.setSetPoint(targetAngle);

        double hSpeed = hPIDDrive.calculate(measuredAngle);
        hSpeed = Range.clip(hSpeed, -hSpeedModifier, hSpeedModifier);

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
        } else {
            if(updateTargetAngle) {
                targetAngle = h;
                updateTargetAngle = false;
            }
        }


        logger.log("pid period", hPIDDrive.getPeriod());

        Vector speed = new Vector(-gamepad.getLeftX()*speedModifier, gamepad.getLeftY()*speedModifier, hSpeed);

       driveSubsystem.driveFieldCentric(speed.x, speed.y, speed.h, h, true);
    }
}
