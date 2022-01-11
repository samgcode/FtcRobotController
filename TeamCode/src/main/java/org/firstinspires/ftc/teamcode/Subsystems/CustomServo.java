package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.Number;

/*
subsystem that controls a single servo in servo mode
 */
public class CustomServo extends SubsystemBase {
    Logger logger;
    SimpleServo servo;

    public CustomServo(Logger logger_, HardwareMap hardwareMap_, String servoName_) {
        logger = logger_;
        servo = new SimpleServo(hardwareMap_, servoName_, 0, 180);
    }


    public void setSpeed(double speed) {
        double input = Number.map(speed, -1, 1, 0, 1);
        servo.setPosition(input);
    }
}
