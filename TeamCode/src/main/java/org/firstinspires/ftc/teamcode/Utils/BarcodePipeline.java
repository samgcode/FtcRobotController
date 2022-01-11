package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;


/*
barcode pipeline
vision pipeline for freight frenzy object detection
 */
@Config
public class BarcodePipeline extends OpenCvPipeline {

    Mat matYCrCb = new Mat();
    public Mat leftBlock = new Mat();
    public Mat middleBlock = new Mat();
    public Mat rightBlock = new Mat();

    public Vector left = new Vector(0, 0);
    public Vector middle = new Vector(0, 0);
    public Vector right = new Vector(0, 0);

    public static int center = 550;
    public static int height = 40;
    public static int spacing = 320;
    public static int vPos = 140;
    public static int width = 50;

    int[] leftRect = {
            (center-spacing)-width, vPos-height,
            (center-spacing)+width, vPos+height};
    int[] middleRect = {
            center-width, vPos-height,
            center+width, vPos+height};
    int[] rightRect = {
            (center+spacing)-width, vPos-height,
            (center+spacing)+width, vPos+height};

    @Override
    public Mat processFrame(Mat input) {
        leftRect = new int[]{
                (center-spacing)-width, vPos-height,
                (center-spacing)+width, vPos+height};
        middleRect = new int[]{
                center-width, vPos-height,
                center+width, vPos+height};
        rightRect = new int[]{
                (center+spacing)-width, vPos-height,
                (center+spacing)+width, vPos+height};

        Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2HSV);

        leftBlock = drawRectangle(matYCrCb, leftRect, new Scalar(0, 0, 0), 5);
        middleBlock = drawRectangle(matYCrCb, middleRect, new Scalar(0, 0, 0), 5);
        rightBlock = drawRectangle(matYCrCb, rightRect, new Scalar(0, 0, 0), 5);

        left = getHS(leftBlock);
        middle = getHS(middleBlock);
        right = getHS(rightBlock);

        return matYCrCb;
    }

    public Mat drawRectangle(Mat frame, int[] points, Scalar color, int thickness) {
        Imgproc.rectangle(
                frame,
                new Point(
                        points[0],
                        points[1]),

                new Point(
                        points[2],
                        points[3]),
                color, thickness);

        return frame.submat(points[1], points[3], points[0], points[2]);
    }

    Vector getHS(Mat block) {
        Scalar mean = Core.mean(block);

        double hue = mean.val[0];
        double saturation = mean.val[1];

        return new Vector(hue, saturation);
    }
}
