package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveToPositionSubsystem extends SubsystemBase {
    MotorEx motor;
    PIDController pidController;

    public DriveToPositionSubsystem(HardwareMap hardwareMap, String motorName, double kP, double kI, double kD) {
        motor = new MotorEx(hardwareMap, motorName);
        pidController = new PIDController(kP, kI, kD);
    }

    public void setPosition(double inches, double tolerance) {
        double TICKS_PER_INCH = 1120;

        pidController.setTolerance(tolerance);
        pidController.setSetPoint(inches*TICKS_PER_INCH);
    }

    public boolean isAtTargetPosition() {
        return pidController.atSetPoint();
    }

    @Override
    public void periodic() {
        double pidOutput = pidController.calculate(motor.getCurrentPosition());

        motor.setVelocity(pidOutput);
    }
}
