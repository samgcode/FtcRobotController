package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Autonomous
public class AutoRed extends CommandOpMode {
    SubsystemLocator subsystemLocator;

    @Override
    public void initialize() {
        subsystemLocator = new SubsystemLocator(new Vector(0,0,0), hardwareMap);
        schedule(new Auto(true, subsystemLocator));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        waitForStart();

        // run the scheduler
        while (!isStopRequested() && opModeIsActive()) {
            run();
        }
        subsystemLocator.stop();
        reset();
    }
}
