package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import org.firstinspires.ftc.teamcode.Utils.BarcodePipeline;
import org.firstinspires.ftc.teamcode.Utils.Vector;

public class VisionSubsystem extends SubsystemBase {
    OpenCvCamera camera;
    BarcodePipeline barcodePipeline;
    public int level;
    public boolean stable = false;

    public VisionSubsystem(HardwareMap hardwareMap) {
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
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
    }


    int counter = 0;
    @Override
    public void periodic() {
        Vector left = barcodePipeline.left;
        Vector middle = barcodePipeline.middle;
        Vector right = barcodePipeline.right;

        level = getMostGreen(new Vector[]{left, middle, right}, 20, 50);

//        if(counter % 20000 == 0) {
//            System.out.println("Frame Count: " + camera.getFrameCount());
//            System.out.println("FPS: " + String.format("%.2f", camera.getFps()));
//            System.out.println("Total frame time ms: " + camera.getTotalFrameTimeMs());
//            System.out.println("Pipeline time ms: " + camera.getPipelineTimeMs());
//            System.out.println("Overhead time ms: " + camera.getOverheadTimeMs());
//            System.out.println("Theoretical max FPS: " + camera.getCurrentPipelineMaxFps());
//            System.out.println("index: " + 0 + ", hue: " + String.format("%.3g", left.x) + ", saturation: " + String.format("%.3g", left.y));
//            System.out.println("index: " + 1 + ", hue: " + String.format("%.3g", middle.x) + ", saturation: " + String.format("%.3g", middle.y));
//            System.out.println("index: " + 2 + ", hue: " + String.format("%.3g", right.x) + ", saturation: " + String.format("%.3g", right.y));
            System.out.println("Position: " + level);
            System.out.println(counter);
//        }
        counter++;
        if(counter >= 30000) {
            stable = true;
        }
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
