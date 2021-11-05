package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;

public class SetElevatorPositionVisionCommand extends SequentialCommandGroup {

    public SetElevatorPositionVisionCommand(Logger logger_, VisionSubsystem visionSubsystem_, HardwareMap hardwareMap_, ContinuousServoSubsystem servo_) {
        addCommands(new SetElevatorPositionCommand(logger_, hardwareMap_, servo_, () -> visionSubsystem_.level));
    }
}
