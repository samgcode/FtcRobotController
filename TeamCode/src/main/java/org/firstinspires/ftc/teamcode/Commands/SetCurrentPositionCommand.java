package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class SetCurrentPositionCommand extends CommandBase {
    HolonomicOdometry holonomicOdometry;
    OdometrySubsystem odometrySubsystem;
    SubsystemLocator subsystemLocator;
    Vector position = new Vector(Double.POSITIVE_INFINITY, 0, 0);
    double heading;

    public SetCurrentPositionCommand(SubsystemLocator subsystemLocator_, double heading_) {
        subsystemLocator = subsystemLocator_;
        holonomicOdometry = subsystemLocator.getHolonomicOdometry();
        odometrySubsystem = subsystemLocator.getOdometrySubsystem();
        heading = heading_;
    }

    public SetCurrentPositionCommand(SubsystemLocator subsystemLocator_, Vector position_) {
        this(subsystemLocator_, position_.h);
        position = position_;
    }

    @Override
    public void initialize() {
        if(Double.isInfinite(position.x)) {
            position = new Vector(odometrySubsystem.getPose());
            position.h = heading;
        }

        subsystemLocator.resetEncoderOffsets(position);
        holonomicOdometry.updatePose(position.toPose2d());
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
