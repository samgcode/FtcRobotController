package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.LogPosition;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Autonomous
public class PurePursuitTesting extends CommandOpMode {
    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    LogPosition logPosition;

    double t = 23.75;//1 tile in inches
    Vector origin, homeA, homeB, hub, carousel, barrier, startingLocation;
    int m = 1;

    @Override
    public void initialize() {
        origin = new Vector(0, 0, 0);
        homeA = new Vector(3*t*m, -1.5*t, -90*m);
        homeB = new Vector(3*t*m, 0.5*t, -90*m);
        hub = new Vector(1.5*t*m, -0.5*t, -90*m);
        carousel = new Vector(2.5*t*m, -2.5*t, -90*m);
        barrier = new Vector(2*t*m, 0.5*t, 0);
        startingLocation = new Vector(3*t*m, -0.5*t, -90*m);

        Vector pointA = new Vector(5, 5, 0);

        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(hardwareMap,
                "motor3", "motor1", "motor2", "motor0");
        odometrySubsystem = SubsystemService.createOdometrySubsystem(hardwareMap,
                "motor2", "motor3", "motor0", origin);

        System.out.println("INIT");

        logPosition = new LogPosition(odometrySubsystem);

        PurePursuitCommand purePursuitCommand = new PurePursuitCommand(driveSubsystem, odometrySubsystem,
                new StartWaypoint(origin.toPose2d()),
                new EndWaypoint(pointA.toPose2d(), 0, 0, 1, 1, 0.05)
        );

        schedule(purePursuitCommand);
    }

}
