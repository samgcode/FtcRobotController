package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Commands.SetElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.SetMechanumSpeedCommand;
import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
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
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Logger logger = new Logger(dashboard);
//
//        TouchSensor[] limitSwitches = new TouchSensor[]{
//                hardwareMap.get(TouchSensor.class, "limit0"),
//                hardwareMap.get(TouchSensor.class, "limit1"),
//                hardwareMap.get(TouchSensor.class, "limit2"),
//                hardwareMap.get(TouchSensor.class, "limit3"),
//                hardwareMap.get(TouchSensor.class, "limit4"),
//        };
//
//        ContinuousServoSubsystem elevatorServo = new ContinuousServoSubsystem(logger, hardwareMap, "servo0", "limit0", "limit4");
//
//        ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem(logger, elevatorServo, limitSwitches);
//
//        subsystemLocator = new SubsystemLocator(
//                hardwareMap, elevatorSubsystem, elevatorServo, gamepad1, gamepad2
//        );

        subsystemLocator = new SubsystemLocator(new Vector(0,0,0), hardwareMap, gamepad1, gamepad2);

        setupGamepadCommands();

        schedule(new SetMechanumSpeedCommand(subsystemLocator));
    }

    void setupGamepadCommands() {
        GamepadEx gamepadEx = new GamepadEx(subsystemLocator.getGamepad1());
        GamepadEx gamepadEx2 = new GamepadEx(subsystemLocator.getGamepad2());

//        Button a = new GamepadButton(gamepadEx, GamepadKeys.Button.A);
        Button b = new GamepadButton(gamepadEx, GamepadKeys.Button.B);
        Button y = new GamepadButton(gamepadEx, GamepadKeys.Button.Y);
        Button x = new GamepadButton(gamepadEx, GamepadKeys.Button.X);

        Button a2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.A);
        Button b2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.B);
        Button y2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.Y);
        Button x2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.X);

        a2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 0));
        b2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 1));
        y2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 3));

    }

}
