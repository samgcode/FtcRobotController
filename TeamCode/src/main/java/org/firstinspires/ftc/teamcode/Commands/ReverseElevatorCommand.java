package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;

public class ReverseElevatorCommand extends CommandBase {
    Logger logger;
    ElevatorSubsystem elevatorSubsystem;

    public ReverseElevatorCommand(SubsystemLocator subsystemLocator) {
        logger = subsystemLocator.getLogger();
        elevatorSubsystem = subsystemLocator.getElevatorSubsystem();

        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
        elevatorSubsystem.reverseSpeed();
    }
}
