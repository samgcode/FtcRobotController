package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoBlue extends CommandOpMode {
    @Override
    public void initialize() {
        schedule(new Auto(false, hardwareMap, telemetry));
    }
}
