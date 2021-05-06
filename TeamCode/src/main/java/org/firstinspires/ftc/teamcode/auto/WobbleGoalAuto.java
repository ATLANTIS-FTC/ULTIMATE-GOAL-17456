package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.WobbleGoalMech;
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

/*
 * This is a simple routine to test translational drive capabilities.
 */
@Config
@Autonomous(group = "drive")
public class WobbleGoalAuto extends LinearOpMode {

    int key = -1;

    private ElapsedTime runtime = new ElapsedTime();


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

    OpenCvCamera phoneCam;

    double strafeFull = -10;
    double forwardFull = -5;
    double turnFull = -20;

    double strafeHalf = 0;
    double forwardHalf = -5;
    double turnHalf = -10;

    double strafeEmpty = -4;
    double forwardEmpty = -5;
    double turnEmpty = -10;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        WobbleGoalMech wobble = new WobbleGoalMech(hardwareMap);
        Shooter shooter = new Shooter();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();//open camera
        phoneCam.setPipeline(new StageSwitchingPipeline());//different stages
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);//display on RC
        //width, height
        //width = height in this case, because camera is in portrait mode.

        telemetry.addData(">", "U R GO, GOOD LUCK!");
        telemetry.update();

        telemetry.addData("Values", valOne+"   "+valTwo+"   "+valThree+"   "+valFour);

        if (valFour == 0) {
            telemetry.addLine("Full");
        } else if (valTwo == 0) {
            telemetry.addLine("Half");
        } else {
            telemetry.addLine("empty");
        }

        waitForStart();

        if (isStopRequested()) return;

        telemetry.addData("Values", valOne+"   "+valTwo+"   "+valThree+"   "+valFour);

        if (valFour == 0) {
            telemetry.addLine("Full");
            key = 0;
        } else if (valTwo == 0) {
            telemetry.addLine("Half");
            key = 1;
        } else {
            telemetry.addLine("empty");
            key = 2;
        }

        telemetry.update();

//        drive.wobbleGoalArm.setPosition(0);


        switch (key) {
            case 0:
                //full stack 13.18
                telemetry.addLine("full");
                telemetry.update();
                Trajectory trajectoryfull = drive.trajectoryBuilder(new Pose2d())
                        .forward(30+forwardFull)
                        .build();
                drive.followTrajectory(trajectoryfull);

                Trajectory trajectoryfull1 = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(70+strafeFull)
                        .build();
                drive.followTrajectory(trajectoryfull1);

                Trajectory trajectoryfull2 = drive.trajectoryBuilder(new Pose2d())
                        .forward(155+forwardFull)
                        .build();
                drive.followTrajectory(trajectoryfull2);
                drive.turn(Math.toRadians(85+turnFull));
                runtime.reset();
                while (runtime.seconds() < 1.2 && opModeIsActive()) {

                }
//                drive.wobbleGoalArm.setPosition(.45);
                drive.wobbleGoalGripper.setPosition(.5);

                drive.turn(Math.toRadians(-(400+turnFull)));
                Trajectory trajectoryfull3 = drive.trajectoryBuilder(new Pose2d())
                        .forward(152+forwardFull)
                        .build();
                drive.followTrajectory(trajectoryfull3);
                drive.turn(Math.toRadians(200+turnFull));

                Trajectory trajectoryfull4 = drive.trajectoryBuilder(new Pose2d())
                        .forward(42+forwardFull)
                        .build();
                drive.followTrajectory(trajectoryfull4);

                runtime.reset();
                while (runtime.seconds() < 1 && opModeIsActive()) {
                    drive.rightFront.setPower(.7);
                    drive.rightRear.setPower(.7);
                }

                Trajectory trajectoryfull5 = drive.trajectoryBuilder(new Pose2d())
                        .forward(152+forwardFull)
                        .build();
                drive.followTrajectory(trajectoryfull5);

                runtime.reset();
                while (runtime.seconds() < 1 && opModeIsActive()) {
                    drive.rightFront.setPower(.7);
                    drive.rightRear.setPower(.7);
                }

                Trajectory trajectoryfull6 = drive.trajectoryBuilder(new Pose2d())
                        .forward(42+forwardFull)
                        .build();
                drive.followTrajectory(trajectoryfull6);

                runtime.reset();
                while (runtime.seconds() < .7 && opModeIsActive()) {
                    drive.rightFront.setPower(-.7);
                    drive.rightRear.setPower(-.7);
                }

                Trajectory trajectoryfull7 = drive.trajectoryBuilder(new Pose2d())
                        .back(68+forwardFull)
                        .build();
                drive.followTrajectory(trajectoryfull7);


                break;
            case 1:
                //half stack 13.07
                telemetry.addLine("half");
                telemetry.update();
                Trajectory trajectoryHalf = drive.trajectoryBuilder(new Pose2d())
                        .forward(160+forwardHalf)
                        .build();
                drive.followTrajectory(trajectoryHalf);

//                drive.wobbleGoalArm.setPosition(.45);
                drive.wobbleGoalGripper.setPosition(.5);

                drive.turn(Math.toRadians(333));
                Trajectory trajectoryhalf2 = drive.trajectoryBuilder(new Pose2d())
                        .forward(150+forwardHalf)
                        .build();
                drive.followTrajectory(trajectoryhalf2);
                drive.turn(Math.toRadians(185+turnHalf));
                runtime.reset();
                while (runtime.seconds() < 1.3 && opModeIsActive()) {
                    drive.rightFront.setPower(.7);
                    drive.rightRear.setPower(.7);
                }

                drive.rightFront.setPower(.0);
                drive.rightRear.setPower(.0);
                Trajectory trajectoryhalf3 = drive.trajectoryBuilder(new Pose2d())
                        .forward(130+forwardHalf)
                        .build();
                drive.followTrajectory(trajectoryhalf3);

                Trajectory trajectoryhalf4 = drive.trajectoryBuilder(new Pose2d())
                        .back(43+forwardHalf)
                        .build();
                drive.followTrajectory(trajectoryhalf4);
                break;
            case 2:
                //empty stack 13
                telemetry.addLine("empty");
                telemetry.update();

                Trajectory trajectoryempty = drive.trajectoryBuilder(new Pose2d())
                        .forward(30+forwardEmpty)
                        .build();
                drive.followTrajectory(trajectoryempty);

                Trajectory trajectoryempty1 = drive.trajectoryBuilder(new Pose2d())
                        .strafeLeft(67+strafeEmpty)
                        .build();
                drive.followTrajectory(trajectoryempty1);

                Trajectory trajectoryempty2 = drive.trajectoryBuilder(new Pose2d())
                        .forward(80+forwardEmpty)
                        .build();
                drive.followTrajectory(trajectoryempty2);
                drive.turn(Math.toRadians(80+turnEmpty));
                runtime.reset();
                while (runtime.seconds() < 1.2 && opModeIsActive()) {

                }
//                drive.wobbleGoalArm.setPosition(.45);
                drive.wobbleGoalGripper.setPosition(.5);

                drive.turn(Math.toRadians(-(389+turnEmpty)));
                Trajectory trajectoryempty3 = drive.trajectoryBuilder(new Pose2d())
                        .forward(80+forwardEmpty)
                        .build();
                drive.followTrajectory(trajectoryempty3);
                drive.turn(Math.toRadians(190));
                Trajectory trajectoryempty4 = drive.trajectoryBuilder(new Pose2d())
                        .forward(47+forwardEmpty)
                        .build();
                drive.followTrajectory(trajectoryempty4);
                runtime.reset();
                while (runtime.seconds() < 1.7 && opModeIsActive()) {
                    drive.rightFront.setPower(.7);
                    drive.rightRear.setPower(.7);
                }

                drive.rightFront.setPower(.0);
                drive.rightRear.setPower(.0);
                Trajectory trajectoryempty5 = drive.trajectoryBuilder(new Pose2d())
                        .forward(69+forwardEmpty)
                        .build();
                drive.followTrajectory(trajectoryempty5);
                break;
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

        private StageSwitchingPipeline.Stage stageToRenderToViewport = StageSwitchingPipeline.Stage.detection;
        private StageSwitchingPipeline.Stage[] stages = StageSwitchingPipeline.Stage.values();

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
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 90, 200, Imgproc.THRESH_BINARY_INV);

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