package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutoRed extends CommandOpMode {
    @Override
    public void initialize() {
        schedule(new Auto(true, hardwareMap, telemetry));
    }
}
