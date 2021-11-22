package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;

@Autonomous
public class VisionTesting extends CommandOpMode {
    VisionSubsystem visionSubsystem;
    Logger logger;
    FtcDashboard dashboard;

    @Override
    public void initialize() {
        dashboard = FtcDashboard.getInstance();
        logger = new Logger();
        visionSubsystem = new VisionSubsystem(logger, hardwareMap);
    }
}
