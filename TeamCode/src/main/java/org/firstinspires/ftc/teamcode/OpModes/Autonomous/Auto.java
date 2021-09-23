package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Utils.Vector;

@Autonomous
public class Auto extends CommandOpMode {
    double t = 23.75;//1 tile in inches

    //positions
    Vector origin = new Vector(0, 0, 0);
    Vector homeA = new Vector(3*t, -1.5*t, -90);
    Vector homeB = new Vector(3*t, 0.5*t, -90);
    Vector hub = new Vector(1.5*t, -0.5*t, -90);
    Vector carousel = new Vector(2.5*t, -2.5*t, -90);
    Vector barrier = new Vector(2*t, 0.5*t, 0);

    @Override
    public void initialize() {

    }

}
