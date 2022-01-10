package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;

import java.util.function.DoubleSupplier;

public class SetElevatorPositionCommand extends CommandBase {
    Logger logger;
    VisionSubsystem visionSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    int targetLevel;
    boolean useVision;

    public SetElevatorPositionCommand(SubsystemLocator subsystemLocator, int level_) {
        logger = subsystemLocator.getLogger();
        elevatorSubsystem = subsystemLocator.getElevatorSubsystem();

        targetLevel = level_;
        addRequirements(elevatorSubsystem);
    }

    public SetElevatorPositionCommand(SubsystemLocator subsystemLocator, boolean useVision_) {
        this(subsystemLocator, -1);
        visionSubsystem = subsystemLocator.getVisionSubsystem();
        useVision = useVision_;
    }

    @Override
    public void initialize() {
        if(targetLevel == -1) {
            targetLevel = visionSubsystem.level;
            if(visionSubsystem.level == -1) {
                targetLevel = 2;
            }
        }

        elevatorSubsystem.targetLevel = targetLevel;
        elevatorSubsystem.manualControl = false;
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isAtTargetLevel();
    }
}
