package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.Utils.Logger;

/*
subsystem to control the intake
 */
public class IntakeSubsystem extends SubsystemBase {
    Logger logger;
    CustomServo motor;
    double speed;

    public IntakeSubsystem(Logger logger_, CustomServo motor_) {
        logger = logger_;
        motor = motor_;
    }

    @Override
    public void periodic() {
        motor.setSpeed(speed);
    }

    public void setSpeed(double speed_) {
        speed = speed_;
    }
}
