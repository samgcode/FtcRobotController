package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.opencv.core.Mat;

import java.util.function.DoubleSupplier;

public class SetIntakeSpeedCommand extends CommandBase {
    Logger logger;
    IntakeSubsystem intakeSubsystem;
    double speed;
    boolean useTriggers;
    Gamepad gamepad;

    public SetIntakeSpeedCommand(SubsystemLocator subsystemLocator, double speed_) {
        logger = subsystemLocator.getLogger();
        intakeSubsystem = subsystemLocator.getIntakeSubsystem();
        speed = speed_;
    }

    public SetIntakeSpeedCommand(SubsystemLocator subsystemLocator, boolean useGamepad) {
        this(subsystemLocator, 0);
        gamepad = subsystemLocator.getGamepad2();
        useTriggers = useGamepad;
    }

    @Override
    public void initialize() {
        intakeSubsystem.setSpeed(speed);
    }

    @Override
    public void execute() {
        if(useTriggers) {
            double f = gamepad.left_trigger;
            double r = gamepad.right_trigger;
            logger.log("f", f);
            logger.log("r", r);

            if(Math.abs(f) > Math.abs(r)) {
                speed = -f;
            } else {
                speed = r;
            }
            intakeSubsystem.setSpeed(speed);
        }
    }

    @Override
    public boolean isFinished() {
        return !useTriggers;
    }
}
