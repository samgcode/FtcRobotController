package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Utils.Logger;

public class ContinuousServoSubsystem extends SubsystemBase {
    Logger logger;
    SimpleServo servo;
    TouchSensor bottom, top;

    public ContinuousServoSubsystem(Logger logger_, HardwareMap hardwareMap_, String servoName_, String bottomLimit, String topLimit) {
        logger = logger_;
        servo = new SimpleServo(hardwareMap_, servoName_, 0, 180);
        bottom = hardwareMap_.get(TouchSensor.class, bottomLimit);
        top = hardwareMap_.get(TouchSensor.class, topLimit);
    }

    @Override
    public void periodic() {
        if(bottom.isPressed() || top.isPressed()) {
            servo.setPosition(0.5);
        }
    }

    public void setSpeed(double speed) {
        servo.setPosition(speed);
    }
}
