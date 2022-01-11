package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;

/*
freight frenzy specific command that uses the vision to determine what level to place freight on in auto
 */
public class FindLevelCommand extends CommandBase {
    Logger logger;
    VisionSubsystem visionSubsystem;
    Timing.Timer timer;
    long time;

    public FindLevelCommand(SubsystemLocator subsystemLocator, long time_) {
        logger = subsystemLocator.getLogger();
        visionSubsystem = subsystemLocator.getVisionSubsystem();
        time = time_;
    }

    @Override
    public void initialize() {
        timer = new Timing.Timer(time);
        timer.start();
    }

    @Override
    public void execute() {
        visionSubsystem.updateVision(timer.done());
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
