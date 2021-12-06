package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utils.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    ContinuousServoSubsystem servo;
    ServoEx bucketServo;
    Logger logger;
    int currentLevel = 0;
    public int targetLevel = 0;
    TouchSensor[] limitSwitches;
    double bucketTarget = 90;

    double minSpeed = 0.5, maxSpeed = 1;

    public ElevatorSubsystem(Logger logger_, ContinuousServoSubsystem servo_, ServoEx bucket_,TouchSensor[] limitSwitches_) {
        servo = servo_;
        bucketServo = bucket_;
        logger = logger_;
        limitSwitches = limitSwitches_;
    }

    @Override
    public void periodic() {
        logger.log("target", targetLevel);
        logger.log("level", currentLevel);
//
        updateCurrentLevel();
        updateServoSpeed();

        bucketServo.turnToAngle(bucketTarget);
    }

    void updateCurrentLevel() {
        logger.log("states", !limitSwitches[0].isPressed() + ", " + !limitSwitches[1].isPressed() + ", " + !limitSwitches[2].isPressed() + ", " + !limitSwitches[3].isPressed());
        for(int limitIndex = 0; limitIndex < limitSwitches.length; limitIndex++) {
            TouchSensor limitSwitch = limitSwitches[limitIndex];
            boolean isPressed = !limitSwitch.isPressed();
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

    public boolean isAtTargetLevel() {
        return (targetLevel == currentLevel);
    }

    public void rotateBucket(double angle) {
        bucketTarget = angle;
    }
}
