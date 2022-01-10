package org.firstinspires.ftc.teamcode.OpModes.DriverControlled;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Commands.RotateBucketCommand;
import org.firstinspires.ftc.teamcode.Commands.RotateCarouselCommand;
import org.firstinspires.ftc.teamcode.Commands.SetElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.Commands.SetIntakeSpeedCommand;
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

        Button a2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.A);
        Button b2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.B);
        Button y2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.Y);
        Button x2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.X);

        Button lb2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.LEFT_BUMPER);
        Button rb2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.RIGHT_BUMPER);

        Button ls2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.LEFT_STICK_BUTTON);
        Button rs2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.RIGHT_STICK_BUTTON);

        Button u2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_UP);
        Button d2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_DOWN);
        Button l2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_LEFT);
        Button r2 = new GamepadButton(gamepadEx2, GamepadKeys.Button.DPAD_RIGHT);

        TriggerReader lt2r = new TriggerReader(gamepadEx2, GamepadKeys.Trigger.LEFT_TRIGGER);
        Trigger lt2 = new Trigger(() -> lt2r.stateJustChanged());
        TriggerReader rt2r = new TriggerReader(gamepadEx2, GamepadKeys.Trigger.RIGHT_TRIGGER);
        Trigger rt2 = new Trigger(() -> rt2r.stateJustChanged());

        //button events
        a2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 0));
        b2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 1));
        y2.whenPressed(new SetElevatorPositionCommand(subsystemLocator, 2));

        lb2.whenPressed(new RotateBucketCommand(subsystemLocator, 170));
        rb2.whenPressed(new RotateBucketCommand(subsystemLocator, 90));

        ls2.whenPressed(new RotateCarouselCommand(subsystemLocator));

        schedule(new SetIntakeSpeedCommand(subsystemLocator, true));
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
