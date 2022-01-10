package org.firstinspires.ftc.teamcode.Commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Subsystems.Odometry;
import org.firstinspires.ftc.teamcode.Utils.Angle;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.firstinspires.ftc.teamcode.Utils.SubsystemLocator;
import org.firstinspires.ftc.teamcode.Utils.Vector;

@Config
public class DriveToPositionCommand extends CommandBase {
    MecanumDrive driveSubsystem;
    Odometry odometrySubsystem;
    Vector targetPosition;
    Logger logger;
    Timing.Timer timer;

    double acceptableErrorXY;
    double acceptableErrorH;
    boolean isFinished = false;
    long time = 10000;

    PIDController xPID;
    PIDController yPID;
    PIDController hPID;

    public static PIDCoefficients xPidCoefficients = new PIDCoefficients(0.2, 0.2, 0.004);
    public static PIDCoefficients yPidCoefficients = new PIDCoefficients(0.2, 0.2, 0.003);
    public static PIDCoefficients hPidCoefficients = new PIDCoefficients(0.008, 0.2, 0.00018);

    public DriveToPositionCommand(SubsystemLocator subsystemLocator, Vector position_, double acceptableErrorXY_, double acceptableErrorH_) {
        logger = subsystemLocator.getLogger();
        driveSubsystem = subsystemLocator.getDriveSubsystem();
        odometrySubsystem = subsystemLocator.getOdometrySubsystem();
        targetPosition = position_;
        acceptableErrorXY = acceptableErrorXY_;
        acceptableErrorH = acceptableErrorH_;

        xPID = new PIDController(0,0,0);
        yPID = new PIDController(0,0,0);
        hPID = new PIDController(0,0,0);
    }

    public DriveToPositionCommand(SubsystemLocator subsystemLocator, Vector position, double acceptableErrorXY, double acceptableErrorH, long timeOutTime) {
        this(subsystemLocator, position, acceptableErrorXY, acceptableErrorH);
        time = timeOutTime;
    }

    @Override
    public void initialize() {
        logger.log("status", "driving to position: " + targetPosition.x + ", " + targetPosition.y + ", " + targetPosition.h);
        isFinished = false;
        timer = new Timing.Timer(time);
        timer.start();
    }

    @Override
    public void execute() {
        double speedModifier = 0.6;

        Vector currentPose = new Vector(odometrySubsystem.getPose());
        double measuredAngle = Angle.getMAngle(currentPose.h, targetPosition.h);

        xPID.setPID(xPidCoefficients.p, xPidCoefficients.i, xPidCoefficients.d);
        yPID.setPID(yPidCoefficients.p, yPidCoefficients.i, yPidCoefficients.d);
        hPID.setPID(hPidCoefficients.p, hPidCoefficients.i, hPidCoefficients.d);

        xPID.setTolerance(acceptableErrorXY);
        yPID.setTolerance(acceptableErrorXY);
        hPID.setTolerance(acceptableErrorH);

        xPID.setSetPoint(targetPosition.x);
        yPID.setSetPoint(targetPosition.y);
        hPID.setSetPoint(targetPosition.h);

        Vector speed = new Vector(
                xPID.calculate(currentPose.x),
                yPID.calculate(currentPose.y),
                hPID.calculate(measuredAngle)
        );

        speed = new Vector(
                Range.clip(speed.x, -speedModifier, speedModifier),
                Range.clip(speed.y, -speedModifier, speedModifier),
                Range.clip(speed.h, -speedModifier, speedModifier)
        );

        if(xPID.atSetPoint() && yPID.atSetPoint() && hPID.atSetPoint()) {
            isFinished = true;
            speed = new Vector(0,0,0);
        }
//        speed = new Vector(0, 0, speed.h);
        driveSubsystem.driveFieldCentric(-speed.y, speed.x, speed.h, currentPose.h);
    }

    @Override
    public boolean isFinished() {
        if(time < 1000) {
            return isFinished || timer.done();
        }
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Vector currentPose = new Vector(odometrySubsystem.getPose());
        driveSubsystem.driveFieldCentric(0, 0, 0, currentPose.h);
    }
}
