package org.firstinspires.ftc.teamcode.OpModes.Autonomous;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.command.PurePursuitCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.CustomOdometrySubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.Utils.SubsystemService;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class Auto extends CommandBase {
    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    VisionSubsystem visionSubsystem;

    double t = 23.75;//1 tile in inches
    Vector origin, homeA, homeB, hub, carousel, barrier, startingLocation;
    int m = 1;

    public Auto(boolean isRed, HardwareMap hardwareMap) {
        if(!isRed) {
            m = -1;
        }

        origin = new Vector(0, 0, 0);
        homeA = new Vector(3*t*m, -1.5*t, -90*m);
        homeB = new Vector(3*t*m, 0.5*t, -90*m);
        hub = new Vector(1.5*t*m, -0.5*t, -90*m);
        carousel = new Vector(2.5*t*m, -2.5*t, -90*m);
        barrier = new Vector(2*t*m, 0.5*t, 0);
        startingLocation = new Vector(3*t*m, -0.5*t, -90*m);

        driveSubsystem = SubsystemService.createMechanumDriveSubsystem(hardwareMap,
                "motor3", "motor1", "motor2", "motor0");
        odometrySubsystem = SubsystemService.createOdometrySubsystem(hardwareMap,
                "motor2", "motor3", "motor0", startingLocation);

        new SequentialCommandGroup(
                new SequentialCommandGroup(//find correct level
                        //point at barcode?
                        //find shipping element position
                ),
                new SequentialCommandGroup(//score freight
                        new PurePursuitCommand(driveSubsystem, odometrySubsystem,
                                new StartWaypoint(startingLocation.toPose2d()),
                                new GeneralWaypoint(hub.toPose2d(), 0, 0, 0)
                        ),
                        new SequentialCommandGroup(//place on correct level
                                //lift arm to level(pass in vision)
                                //deposit freight
                        )
                ),
                new SequentialCommandGroup(//score carousel
                        new PurePursuitCommand(driveSubsystem, odometrySubsystem,
                                new StartWaypoint(hub.toPose2d()),
                                new GeneralWaypoint(carousel.toPose2d(), 0, 0, 0)
                        )
                        //turn carousel
                ),
                new SequentialCommandGroup(//park in warehouse
                        new PurePursuitCommand(driveSubsystem, odometrySubsystem,
                                new StartWaypoint(carousel.toPose2d()),
                                new GeneralWaypoint(barrier.toPose2d(), 0, 0, 0)
                        )
                        //drive straight for 5 sec
                )
        );
    }
}

/*
auto
>>score preloaded freight

>>>>find correct level
>>>>>>point toward barcode?
>>>>>>find shipping element position

>>>>put freight on correct level (vision subsystem)
>>>>>>move to hub
>>>>>>place freight on level X
>>>>>>>>lift arm to level X
>>>>>>>>deposit freight
>>>>>>>>lower arm

>>score carousel
>>>>move to carousel
>>>>turn carousel

>>park in warehouse
>>>>move to barrier
>>>>drive straight for 5 sec
*/