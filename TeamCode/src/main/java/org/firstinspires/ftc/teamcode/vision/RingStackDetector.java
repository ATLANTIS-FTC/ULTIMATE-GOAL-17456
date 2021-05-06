package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvInternalCamera2;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

public class RingStackDetector {

    OpenCvCamera phoneCam;
    int ringLevel = -1;
    Telemetry telemetry;
    HardwareMap hwmp;
    double rows, rect1Cols, rect2Cols;
    double lowValue;
    double topValue;
    double thresh = 100;


    public RingStackDetector(HardwareMap hw, Telemetry telemetry, double rows, double rect1Cols, double rect2Cols){
        this.hwmp = hw;
        this.telemetry = telemetry;
        this.rows = rows;
        this.rect1Cols = rect1Cols;
        this.rect2Cols = rect2Cols;
    }

    public void init() {
        int cameraMonitorViewId = hwmp.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwmp.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.setPipeline(new RingStackDetector.RingPipeline());
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }

    public int getRingLevel() {
        return ringLevel;
    }

    class RingPipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            input.convertTo(input, -1,150,20);
            return input;
        }
    }
}
