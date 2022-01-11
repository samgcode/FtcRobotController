package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Utils.Logger;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Utils.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Utils.Vector;

/*
subsystem that feeds the camera into the vision pipeline and uses the data to figure out where an object is
 */
public class VisionSubsystem extends SubsystemBase {
    OpenCvCamera camera;
    BarcodePipeline barcodePipeline;
    Logger logger;
    public int level;
    public boolean stable = false;

    public VisionSubsystem(Logger logger_, HardwareMap hardwareMap) {
        logger = logger_;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        barcodePipeline = new BarcodePipeline();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.setPipeline(barcodePipeline);

                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                System.out.println(errorCode);
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera, 0);
    }

    public void updateVision(boolean update) {
        if(!update) {
            Vector left = barcodePipeline.left;
            Vector middle = barcodePipeline.middle;
            Vector right = barcodePipeline.right;

            level = getMostGreen(new Vector[]{left, middle, right}, 20, 50);

            logHS(0, left);
            logHS(1, middle);
            logHS(2, right);
        }
        logger.log("Detected position", level);
    }

    void logHS(int index, Vector values) {
        logger.log("hue " + index, String.format("%.3g", values.x));
        logger.log("saturation " + index, String.format("%.3g", values.y));
    }

    int getMostGreen(Vector[] values, double min, double max) {
        double biggest = Double.NEGATIVE_INFINITY;
        int index = -1;
        for(int i = 0; i < values.length; i++) {
            Vector value = values[i];
            double hue = value.x;
            double saturation = value.y;

            if(hue < max && hue > min) {
                if(saturation > biggest) {
                    biggest = saturation;
                    index = i;
                }
            }
        }
        return index;
    }
}
