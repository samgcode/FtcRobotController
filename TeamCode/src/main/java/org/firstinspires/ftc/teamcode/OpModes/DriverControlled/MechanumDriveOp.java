package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.RotateCarouselCommand;
import org.firstinspires.ftc.teamcode.Commands.SetElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.SetMechanumSpeedCommand;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@TeleOp(name="Mechaunum drive", group="drive")
public class MechanumDriveOp extends CommandOpMode {
    SubsystemLocator subsystemLocator;

    @Override
    public void initialize() {

        subsystemLocator = new SubsystemLocator(new Vector(0,0,0), hardwareMap, gamepad1, gamepad2);

        setupGamepadCommands();

        schedule(new SetMechanumSpeedCommand(subsystemLocator));
    }

    void setupGamepadCommands() {
        GamepadEx gamepadEx = new GamepadEx(subsystemLocator.getGamepad1());
        GamepadEx gamepadEx2 = new GamepadEx(subsystemLocator.getGamepad2());

        Button a = new GamepadButton(gamepadEx, GamepadKeys.Button.A);
        Button b = new GamepadButton(gamepadEx, GamepadKeys.Button.B);
        Button y = new GamepadButton(gamepadEx, GamepadKeys.Button.Y);
        Button x = new GamepadButton(gamepadEx, GamepadKeys.Button.X);

        Button lb = new GamepadButton(gamepadEx, GamepadKeys.Button.LEFT_BUMPER);
        Button rb = new GamepadButton(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER);

        Button a2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.A);
        Button b2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.B);
        Button y2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.Y);
        Button x2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.X);

        Button lb2 = new GamepadButton(gamepadEx, GamepadKeys.Button.LEFT_BUMPER);
        Button rb2 = new GamepadButton(gamepadEx, GamepadKeys.Button.RIGHT_BUMPER);

        a2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 0));
        b2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 1));
        y2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 2));
        x2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 3));

        rb2.whenPressed(new RotateCarouselCommand(subsystemLocator));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        subsystemLocator.stop();
        reset();
    }

}
