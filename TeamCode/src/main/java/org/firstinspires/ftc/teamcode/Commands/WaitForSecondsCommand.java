package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;

public class WaitForSecondsCommand extends CommandBase {
    Timing.Timer timer;
    Logger logger;
    long time;

    public WaitForSecondsCommand(SubsystemLocator subsystemLocator, double time_) {
        logger = subsystemLocator.getLogger();
        time = (long)time_;
    }

    @Override
    public void initialize() {
        timer = new Timing.Timer(time);
        timer.start();
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
