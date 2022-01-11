package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

/*
command that resets the current measured position of the robot to account for encoder inaccuracies
 */
public class SetCurrentPositionCommand extends CommandBase {
    HolonomicOdometry holonomicOdometry;
    Odometry odometrySubsystem;
    Logger logger;
    SubsystemLocator subsystemLocator;
    Vector position = new Vector(Double.POSITIVE_INFINITY, 0, 0);
    double heading;

    public SetCurrentPositionCommand(SubsystemLocator subsystemLocator_, double heading_) {
        subsystemLocator = subsystemLocator_;
        holonomicOdometry = subsystemLocator.getHolonomicOdometry();
        odometrySubsystem = subsystemLocator.getOdometrySubsystem();
        logger = subsystemLocator.getLogger();
        heading = heading_;
    }

    public SetCurrentPositionCommand(SubsystemLocator subsystemLocator_, Vector position_) {
        this(subsystemLocator_, position_.h);
        position = position_;
    }

    @Override
    public void initialize() {
        logger.log("status", "updating current position");
        if(Double.isInfinite(position.x)) {
            position = new Vector(odometrySubsystem.getPose());
            position.h = heading;
        }

        subsystemLocator.resetEncoderOffsets(position);
        odometrySubsystem.updatePose(position.toPose2d());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
