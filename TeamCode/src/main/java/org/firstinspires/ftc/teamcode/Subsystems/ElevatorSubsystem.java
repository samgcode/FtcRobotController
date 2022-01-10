package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utils.Logger;

public class ElevatorSubsystem extends SubsystemBase {
    ContinuousServoSubsystem servo;
    ServoEx bucketServo;
    Logger logger;
    int currentLevel = 1;
    public int targetLevel = 0;
    TouchSensor[] limitSwitches;
    double bucketTarget = 90;
    public boolean manualControl;
    double speed = 0.5;

    double minSpeed = 0.5, maxSpeed = 1;

    public ElevatorSubsystem(Logger logger_, ContinuousServoSubsystem servo_, ServoEx bucket_,TouchSensor[] limitSwitches_) {
        servo = servo_;
        bucketServo = bucket_;
        logger = logger_;
        limitSwitches = limitSwitches_;
    }

    @Override
    public void periodic() {
        constrainBucket();
        logger.log("manual", manualControl);
        if(manualControl) {
            servo.setSpeed(speed);
        } else {
            logger.log("target", targetLevel);
            logger.log("level", currentLevel);

            updateCurrentLevel();
            updateServoSpeed();
            if(currentLevel == targetLevel) { manualControl = true; }
        }
    }

    void constrainBucket() {
        if(targetLevel < currentLevel) {
            bucketTarget = 45;
        }

        if(targetLevel == 0 && currentLevel == 1) {
            bucketTarget = 90;
        }
        if(currentLevel == 0 && bucketTarget >= 170) {
            bucketTarget = 170;
        }
        bucketServo.turnToAngle(bucketTarget);
    }

    void updateCurrentLevel() {
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

    public void setSpeed(double speed_) {
        speed = speed_;
    }
}
