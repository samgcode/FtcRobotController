package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Subsystems.ContinuousServoSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;

import java.util.function.DoubleSupplier;

public class SetElevatorPositionCommand extends CommandBase {
    Logger logger;
    TouchSensor[] limitSwitches;
    ContinuousServoSubsystem servo;
    int targetLevel, currentLevel;
    DoubleSupplier levelSupplier;

    double minSpeed = 0.5, maxSpeed = 1;

    public SetElevatorPositionCommand(Logger logger_, HardwareMap hardwareMap_, ContinuousServoSubsystem servo_, int level_) {
        logger = logger_;
        targetLevel = level_;
        limitSwitches = new TouchSensor[]{
                hardwareMap_.get(TouchSensor.class, "limit0"),
                hardwareMap_.get(TouchSensor.class, "limit1"),
                hardwareMap_.get(TouchSensor.class, "limit2"),
                hardwareMap_.get(TouchSensor.class, "limit3"),
                hardwareMap_.get(TouchSensor.class, "limit4"),
        };
        servo = servo_;
    }

    public SetElevatorPositionCommand(Logger logger_, HardwareMap hardwareMap_, ContinuousServoSubsystem servo_, DoubleSupplier levelSupplier_) {
        this(logger_, hardwareMap_, servo_, -1);
        levelSupplier = levelSupplier_;
    }

    @Override
    public void initialize() {
        if(targetLevel == -1) {
            targetLevel = (int)levelSupplier.getAsDouble();
        }
    }

    @Override
    public void execute() {
        logger.log("btn states",
                limitSwitches[0].isPressed() + ", " +
                     limitSwitches[1].isPressed() + ", " +
                     limitSwitches[2].isPressed() + ", " +
                     limitSwitches[3].isPressed() + ", " +
                     limitSwitches[4].isPressed());

        updateCurrentLevel();
        updateServoSpeed();
    }

    void updateCurrentLevel() {
        for(int limitIndex = 0; limitIndex < limitSwitches.length; limitIndex++) {
            TouchSensor limitSwitch = limitSwitches[limitIndex];
            if(limitSwitch.isPressed()) {
                currentLevel = limitIndex;
            }
        }
    }

    void updateServoSpeed() {
        if(currentLevel < targetLevel) {
            servo.setSpeed(maxSpeed);
        } else if(currentLevel > targetLevel) {
            servo.setSpeed(-maxSpeed);
        } else {
            servo.setSpeed(minSpeed);
        }
    }
}
