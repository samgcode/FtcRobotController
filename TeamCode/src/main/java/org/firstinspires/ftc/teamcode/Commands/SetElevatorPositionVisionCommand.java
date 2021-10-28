package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Subsystems.DriveToPositionSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

public class SetElevatorPositionVisionCommand extends SequentialCommandGroup {
    double[] positions = new double[]{5.5, 10, 16};

    public SetElevatorPositionVisionCommand(VisionSubsystem visionSubsystem, DriveToPositionSubsystem driveToPositionSubsystem, double tolerance) {
        addCommands(
                new SetMotorPositionCommand(driveToPositionSubsystem, positions[visionSubsystem.level], tolerance)
        );

        addRequirements(driveToPositionSubsystem);
    }
}
