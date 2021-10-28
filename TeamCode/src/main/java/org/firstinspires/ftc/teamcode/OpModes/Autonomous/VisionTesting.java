package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;

@Autonomous
public class VisionTesting extends CommandOpMode {
    VisionSubsystem visionSubsystem;

    @Override
    public void initialize() {
        visionSubsystem = new VisionSubsystem(hardwareMap);
    }
}
