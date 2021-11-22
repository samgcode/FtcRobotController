package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utils.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    ContinuousServoSubsystem servo;
    Logger logger;
    int currentLevel = 0;
    public int targetLevel = 0;
    TouchSensor[] limitSwitches;

    double minSpeed = 0.5, maxSpeed = 1;

    public ElevatorSubsystem(Logger logger_, ContinuousServoSubsystem servo_, TouchSensor[] limitSwitches_) {
        servo = servo_;
        logger = logger_;
        limitSwitches = limitSwitches_;
    }

    @Override
    public void periodic() {
//        logger.log("target", targetLevel);
//        logger.log("level", currentLevel);
//
//        updateCurrentLevel();
//        updateServoSpeed();
    }

    void updateCurrentLevel() {
        for(int limitIndex = 0; limitIndex < limitSwitches.length; limitIndex++) {
            TouchSensor limitSwitch = limitSwitches[limitIndex];
            boolean isPressed = !limitSwitch.isPressed();
            if(limitIndex >= 4) {
                isPressed = !isPressed;
            }
            if(isPressed) {
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

    public boolean isAtTarget() {
        return true;
//        return (targetLevel == currentLevel);
    }
}
