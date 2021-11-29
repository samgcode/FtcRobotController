package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Autonomous
public class AutoBlue extends CommandOpMode {
    @Override
    public void initialize() {
        SubsystemLocator subsystemLocator = new SubsystemLocator(new Vector(0,0,0), hardwareMap);
        schedule(new Auto(false, subsystemLocator));
    }
}
