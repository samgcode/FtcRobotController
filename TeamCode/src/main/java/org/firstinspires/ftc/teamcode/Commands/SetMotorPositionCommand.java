package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Subsystems.DriveToPositionSubsystem;

public class SetMotorPositionCommand extends CommandBase {
    DriveToPositionSubsystem driveToPositionSubsystem;
    double inches;
    double tolerance;

    public SetMotorPositionCommand(DriveToPositionSubsystem driveToPositionSubsystem_, double inches_, double tolerance_) {
        driveToPositionSubsystem = driveToPositionSubsystem_;
        inches = inches_;
        tolerance = tolerance_;

        addRequirements(driveToPositionSubsystem);
    }

    @Override
    public void execute() {
        driveToPositionSubsystem.setPosition(inches, tolerance);
    }

    @Override
    public boolean isFinished() {
        return driveToPositionSubsystem.isAtTargetPosition();
    }
}
