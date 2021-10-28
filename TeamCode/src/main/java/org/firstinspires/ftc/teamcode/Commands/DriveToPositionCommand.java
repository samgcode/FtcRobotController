package org.firstinspires.ftc.teamcode.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utils.Vector;

public class DriveToPositionCommand extends CommandBase {
    MecanumDrive driveSubsystem;
    OdometrySubsystem odometrySubsystem;
    Vector targetPosition;
    double acceptableErrorXY;
    double acceptableErrorH;
    boolean isFinished = false;

    public DriveToPositionCommand(MecanumDrive driveSubsystem_, OdometrySubsystem odometrySubsystem_, Vector position_, double acceptableErrorXY_, double acceptableErrorH_) {
        driveSubsystem = driveSubsystem_;
        odometrySubsystem = odometrySubsystem_;
        targetPosition = position_;
        acceptableErrorXY = acceptableErrorXY_;
        acceptableErrorH = acceptableErrorH_;

        addRequirements(odometrySubsystem);
    }

    @Override
    public void initialize() {
        isFinished = false;
    }

    @Override
    public void execute() {
        double speedModifier = 0.5;

        Vector currentPose = new Vector(odometrySubsystem.getPose());
        double measuredAngle = getMAngle(currentPose.h, targetPosition.h);

        PIDController xPID = new PIDController(0.1, 0, 0);
        PIDController yPID = new PIDController(0.1, 0, 0);
        PIDController hPID = new PIDController(0.025, 1, 0.15);

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
        System.out.println(speed.h);
//        speed = new Vector(0,0,0);

        driveSubsystem.driveFieldCentric(-speed.y, speed.x, speed.h, currentPose.h);
    }

    double getMAngle(double mAngle, double targetAngle) {
        double difference = targetAngle-mAngle;

        if(difference < -180) {
            difference = 360+difference;
        } else if(difference > 180) {
            difference = 360-difference;
            difference *= -1;
        }

        return targetAngle - difference;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Vector currentPose = new Vector(odometrySubsystem.getPose());
        driveSubsystem.driveFieldCentric(0, 0, 0, currentPose.h);
    }
}
