package org.firstinspires.ftc.teamcode.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.Logger;

/*
opmode for testing vision code
 */
@Disabled
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
