package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Utils.Logger;

/*
subsystem to control the elevator based on gamepad inputs
 */
public class DriveElevatorSubsystem extends SubsystemBase {
    Logger logger;
    ButtonReader upButton, downButton;
    ElevatorSubsystem elevator;

    public DriveElevatorSubsystem(Logger logger_, Gamepad gamepad_, ElevatorSubsystem elevatorSubsystem) {
        logger = logger_;
        GamepadEx gamepadEx = new GamepadEx(gamepad_);
        elevator = elevatorSubsystem;

        upButton = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_UP);
        downButton = new ButtonReader(gamepadEx, GamepadKeys.Button.DPAD_DOWN);
    }

    @Override
    public void periodic() {
        if(upButton.isDown()) {
            elevator.setSpeed(1);
        } else if(downButton.isDown()) {
            elevator.setSpeed(0);
        } else {
            elevator.setSpeed(0.5);
        }
    }
}
