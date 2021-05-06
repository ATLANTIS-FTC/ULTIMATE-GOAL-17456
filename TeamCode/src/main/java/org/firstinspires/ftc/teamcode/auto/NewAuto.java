package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.OpenCVTestingGround;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.lang.Math;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class NewAuto extends LinearOpMode {

    //0 means skystone, 1 means yellow stone
    //-1 for debug, but we can keep it like this because if it works, it should change to either 0 or 255
    private static int valOne = -1;
    private static int valTwo = -1;
    private static int valThree = -1;
    private static int valFour = -1;

    private static float rectHeight = .2f/8f;
    private static float rectWidth = 1.1f/8f;

    private static float offsetX = -1f/8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = -.5f/8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] onePos = {4.75f/8f+offsetX, 6.5f/8f+offsetY};//0 = col, 1 = row
    private static float[] twoPos = {4.75f/8f+offsetX, 6.3f/8f+offsetY};
    private static float[] threePos = {4.75f/8f+offsetX, 6.1f/8f+offsetY};
    private static float[] fourPos = {4.75f/8f+offsetX, 5.9f/8f+offsetY};
    //moves all rectangles right or left by amount. units are in ratio to monitor

    private final int rows = 640;
    private final int cols = 480;

    int key = -1;

    OpenCvCamera phoneCam;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ElapsedTime runtime = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        //generate paths
        Trajectory dropWobbleB = drive.trajectoryBuilder(new Pose2d(-60,35), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-25, 55), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(27,35), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "1/7");
        telemetry.update();

        Trajectory park = drive.trajectoryBuilder(dropWobbleB.end())
                .lineTo(new Vector2d(15,35))
                .build();
        telemetry.addData("PathGen:", "2/7");
        telemetry.update();

        Trajectory dropWobbleA = drive.trajectoryBuilder(new Pose2d(-60,35), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(10, 60), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "3/7");
        telemetry.update();

        Trajectory dropWobbleCont = drive.trajectoryBuilder(dropWobbleA.end())
                .lineTo(new Vector2d(53, 46))
                .build();
        telemetry.addData("PathGen:", "4/7");
        telemetry.update();

        Trajectory park3 = drive.trajectoryBuilder(dropWobbleCont.end())
                .forward(2)
                .build();
        telemetry.addData("PathGen:", "5/7");
        telemetry.update();

        Trajectory park2 = drive.trajectoryBuilder(dropWobbleCont.end())
                .back(35)
                .build();
        telemetry.addData("PathGen:", "5/7");
        telemetry.update();

        //let drive team know its safe to start the program
        telemetry.addData(">", "U R GO, GOOD LUCK!");
        telemetry.update();

        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valOne + "   " + valTwo + "   " + valThree + "   " + valFour);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            if (valFour == 0) {
                key = 0;
                telemetry.addLine("Full");
            } else if (valOne == 0) {
                key = 0;
                telemetry.addLine("Half");
            } else {
                key = 0;
                telemetry.addLine("empty");
            }

            telemetry.update();
            sleep(100);

            if (isStopRequested()) return;

//            drive.wobbleGoalArm.setPosition(0);
            drive.setPoseEstimate(new Pose2d(-60,35,0));

            switch (key) {
                case 0:
                    // empty stack

                    drive.followTrajectory(dropWobbleA);
//                    drive.wobbleGoalArm.setPosition(.45);
                    sleep(2000);
//                    drive.wobbleGoalArm.setPosition(.1);
                    sleep(2000);
                    // drive.followTrajectory(park3);
//                    drive.wobbleGoalArm.setPosition(.35);

                    break;
                case 1:
                    // half stack

                    drive.followTrajectory(dropWobbleB);
//                    drive.wobbleGoalArm.setPosition(.45);
                    sleep(2000);
//                    drive.wobbleGoalArm.setPosition(.1);
                    sleep(2000);
                    drive.followTrajectory(park);


//                    drive.shooterOne.setPower(1);
//                    drive.shooterTwo.setPower(1);
//
//                    drive.followTrajectory(powerShot);
//                    sleep(1500);
//                    drive.spindexer.setPower(-1);



//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(new Pose2d())
//                                    .splineTo(new Vector2d(-40, 35), 0)
//                                    .build()
//                    );
//
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(new Pose2d())
//                                    .splineTo(new Vector2d(-40, 24), 0)
//                                    .build()
//                    );
//
//                    drive.wobbleGoalGripper.setPosition(.5);
//                    drive.wobbleGoalArm.setPosition(.45);
//
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(new Pose2d())
//                                    .splineTo(new Vector2d(35, 25), -90)
//                                    .build()
//                    );
//
//                    drive.wobbleGoalArm.setPosition(.1);
//                    drive.wobbleGoalGripper.setPosition(0);
//
//                    drive.followTrajectory(
//                            drive.trajectoryBuilder(new Pose2d())
//                                    .splineTo(new Vector2d(10, 0), 180)
//                                    .build()
//                    );
                    break;
                case 2:
                    // full stack

                    drive.followTrajectory(dropWobbleA);
                    drive.followTrajectory(dropWobbleCont);
//                    drive.wobbleGoalArm.setPosition(.45);
                    drive.followTrajectory(park2);

            }
            stop();
        /*drive.wobbleGoalArm.setPosition(0);

        drive.setPoseEstimate(new Pose2d(-60,35,0));

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(0, 60), 20)
                .build();
        drive.followTrajectory(traj);

        drive.wobbleGoalArm.setPosition(.45);

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(-50, 0), 180)
                .build();
        drive.followTrajectory(traj2);

        sleep(2000); *?

//        drive.followTrajectory(
//                drive.trajectoryBuilder(traj.end(), true)
//                        .splineTo(new Vector2d(35, 70), Math.toRadians(-90))
//                        .build()
//        );
        /* 35 35 heading 0  for middle wobble goal
        60 60 -90 heading
        12 60 heading 0
         */
        }
    }
    //detection pipeline
    static class StageSwitchingPipeline extends OpenCvPipeline
    {
        Mat yCbCrChan2Mat = new Mat();
        Mat thresholdMat = new Mat();
        Mat all = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();

        enum Stage
        {//color difference. greyscale
            detection,//includes outlines
            THRESHOLD,//b&w
            RAW_IMAGE,//displays raw view
        }

        private Stage stageToRenderToViewport = Stage.detection;
        private Stage[] stages = Stage.values();

        @Override
        public void onViewportTapped()
        {
            /*
             * Note that this method is invoked from the UI thread
             * so whatever we do here, we must do quickly.
             */

            int currentStageNum = stageToRenderToViewport.ordinal();

            int nextStageNum = currentStageNum + 1;

            if(nextStageNum >= stages.length)
            {
                nextStageNum = 0;
            }

            stageToRenderToViewport = stages[nextStageNum];
        }

        @Override
        public Mat processFrame(Mat input)
        {
            contoursList.clear();
            /*
             * This pipeline finds the contours of yellow blobs
             */

            //color diff cb.
            //lower cb = more blue = skystone = white
            //higher cb = less blue = yellow stone = grey
            Imgproc.cvtColor(input, yCbCrChan2Mat, Imgproc.COLOR_RGB2YCrCb);//converts rgb to ycrcb
            Core.extractChannel(yCbCrChan2Mat, yCbCrChan2Mat, 2);//takes cb difference and stores

            //b&w
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 105, 200, Imgproc.THRESH_BINARY_INV);

            //outline/contour
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            yCbCrChan2Mat.copyTo(all);//copies mat object
            //Imgproc.drawContours(all, contoursList, -1, new Scalar(255, 0, 0), 3, 8);//draws blue contours


            //get values from frame
            double[] pixTwo = thresholdMat.get((int)(input.rows()* twoPos[1]), (int)(input.cols()* twoPos[0]));//gets value at circle
            valTwo = (int)pixTwo[0];

            double[] pixOne = thresholdMat.get((int)(input.rows()* onePos[1]), (int)(input.cols()* onePos[0]));//gets value at circle
            valOne = (int)pixOne[0];

            double[] pixThree = thresholdMat.get((int)(input.rows()* threePos[1]), (int)(input.cols()* threePos[0]));//gets value at circle
            valThree = (int)pixThree[0];

            double[] pixFour = thresholdMat.get((int)(input.rows()* fourPos[1]), (int)(input.cols()* fourPos[0]));//gets value at circle
            valFour = (int)pixFour[0];

            //create three points
            Point pointTwo = new Point((int)(input.cols()* twoPos[0]), (int)(input.rows()* twoPos[1]));
            Point pointOne = new Point((int)(input.cols()* onePos[0]), (int)(input.rows()* onePos[1]));
            Point pointThree = new Point((int)(input.cols()* threePos[0]), (int)(input.rows()* threePos[1]));
            Point pointFour = new Point((int)(input.cols()* fourPos[0]), (int)(input.rows()* fourPos[1]));

            //draw circles on those points
            Imgproc.circle(all, pointTwo,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointOne,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointThree,5, new Scalar( 255, 0, 0 ),1 );//draws circle
            Imgproc.circle(all, pointFour,5, new Scalar( 255, 0, 0 ),1 );//draws circle

            //draw 3 rectangles
            Imgproc.rectangle(//1-3
                    all,
                    new Point(
                            input.cols()*(onePos[0]-rectWidth/2),
                            input.rows()*(onePos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(onePos[0]+rectWidth/2),
                            input.rows()*(onePos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//3-5
                    all,
                    new Point(
                            input.cols()*(twoPos[0]-rectWidth/2),
                            input.rows()*(twoPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(twoPos[0]+rectWidth/2),
                            input.rows()*(twoPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(threePos[0]-rectWidth/2),
                            input.rows()*(threePos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(threePos[0]+rectWidth/2),
                            input.rows()*(threePos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);
            Imgproc.rectangle(//5-7
                    all,
                    new Point(
                            input.cols()*(fourPos[0]-rectWidth/2),
                            input.rows()*(fourPos[1]-rectHeight/2)),
                    new Point(
                            input.cols()*(fourPos[0]+rectWidth/2),
                            input.rows()*(fourPos[1]+rectHeight/2)),
                    new Scalar(0, 255, 0), 3);

            switch (stageToRenderToViewport)
            {
                case THRESHOLD:
                {
                    return thresholdMat;
                }

                case detection:
                {
                    return all;
                }

                case RAW_IMAGE:
                {
                    return input;
                }

                default:
                {
                    return input;
                }
            }
        }

    }
}

