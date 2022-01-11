package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;

/*
robot specific command to handle rotating the bucket
 */
public class RotateBucketCommand extends CommandBase {
    Logger logger;
    ElevatorSubsystem elevatorSubsystem;
    double angle;
    Timing.Timer timer;

    public RotateBucketCommand(SubsystemLocator subsystemLocator, double angle_) {
        logger = subsystemLocator.getLogger();
        elevatorSubsystem = subsystemLocator.getElevatorSubsystem();
        angle = angle_;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        timer = new Timing.Timer(2);
        timer.start();
        elevatorSubsystem.rotateBucket(angle);
        logger.log("status", "rotating bucket");
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}
