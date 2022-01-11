package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
implementation of the FTCLib motor that allows for changing feed forward coeffcients
 */
public class FeedForwardMotor extends Motor {

    public FeedForwardMotor(HardwareMap hardwareMap, String name) {
        super(hardwareMap, name);
    }

    @Override
    public void set(double output) {
        motor.setPower(feedforward.calculate(output));
    }
}
