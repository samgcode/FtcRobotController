package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing.Timer;

import org.firstinspires.ftc.teamcode.Subsystems.CustomServo;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;

@Config
public class RotateCarouselCommand extends CommandBase {
    Logger logger;
    CustomServo servo;
    Timer timer;

    boolean isFinished;
    public static double speed = 0.2;

    public RotateCarouselCommand(SubsystemLocator subsystemLocator) {
        isFinished = false;
        logger = subsystemLocator.getLogger();
        servo = subsystemLocator.getCarouselServo();
    }

    @Override
    public void initialize() {
        timer = new Timer(3);
        timer.start();
    }

    @Override
    public void execute() {
        if(!timer.done()) {
            servo.setSpeed(speed);
        } else {
            servo.setSpeed(0);
            isFinished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
