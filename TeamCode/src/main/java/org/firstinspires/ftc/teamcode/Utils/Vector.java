package org.firstinspires.ftc.teamcode.Utils;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class Vector {
    public double x, y, z, h;

    public Vector(double x_, double y_) {
        x = x_;
        y = y_;
    }

    public Vector(double x_, double y_, double z_) {
        x = x_;
        y = y_;
        h = z_;
        z = z_;
    }

    public Vector(Pose2d pose) {
        x = pose.getX();
        y = pose.getY();
        h = pose.getHeading()*(180/Math.PI);
        z = pose.getHeading()*(180/Math.PI);
    }

    public static Vector normalizeVector(Vector vector) {
        double angle = Math.toRadians(vector.h);
        double yComponent = (-(vector.x * Math.sin(angle)) + (vector.y * Math.cos(angle)));
        double xComponent = ((vector.x * Math.cos(angle)) + (vector.y * Math.sin(angle)));

        return new Vector(xComponent, yComponent);
    }

    public Pose2d toPose2d() {
        return new Pose2d(this.x, this.y, new Rotation2d((this.h*(Math.PI/180))));
    }
}