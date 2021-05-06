package org.firstinspires.ftc.teamcode.auto;

import android.graphics.Path;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.exception.MathParseException;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.util.PoseStorage;
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

import TeleOp.NovaOp;

@Autonomous(group = "Feb")
public class HighGoalWobbleAuto extends LinearOpMode {

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


    int key = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter();
        ElapsedTime runtime = new ElapsedTime();

        ElapsedTime veloTimer = new ElapsedTime();

        drive.shooterOne.setDirection(DcMotorSimple.Direction.FORWARD);
        drive.shooterTwo.setDirection(DcMotorSimple.Direction.FORWARD);

        drive.spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER, drive);
        double kP = 1200;
        double kD = 15;
        double kF = 10;

        PIDFCoefficients pidfCoeffs = new PIDFCoefficients(kP, 0, kD, kF);
        drive.shooterOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
        drive.shooterTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);



        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new StageSwitchingPipeline());
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);

        Trajectory highGoal = drive.trajectoryBuilder(new Pose2d(-64,43), Math.toRadians(0))
                .lineTo(new Vector2d(-3.4, 44))
                .build();
        telemetry.addData("PathGen:", "1/18");
        telemetry.update();

        Trajectory highGoalBAndC = drive.trajectoryBuilder(new Pose2d(-64,43), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, 60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0,60), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(0,40), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "1/18");
        telemetry.update();

        Trajectory dropWobbleOneA = drive.trajectoryBuilder(highGoal.end())
                .lineToLinearHeading(new Pose2d(-2, 56, Math.toRadians(60)))
                .build();
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();

        Trajectory dropWobbleOneC = drive.trajectoryBuilder(highGoal.end())
                .lineToLinearHeading(new Pose2d(45, 56.5, Math.toRadians(60)))
                .build();
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();

        Trajectory pickWobbleTwoC = drive.trajectoryBuilder(dropWobbleOneC.end())
                .lineToLinearHeading(new Pose2d(-41,44, Math.toRadians(-90)))
                .build();
        telemetry.addData("PathGen:", "8/18");
        telemetry.update();

        Trajectory pickWobbleTwoCCont = drive.trajectoryBuilder(pickWobbleTwoC.end())
                .lineToLinearHeading(new Pose2d(-35,25, Math.toRadians(-90)))
                .build();
        telemetry.addData("PathGen:", "9/18");
        telemetry.update();

        Trajectory dropWobbleTwoC = drive.trajectoryBuilder(pickWobbleTwoCCont.end())
                .lineToLinearHeading(new Pose2d(47, 53, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();


        Trajectory pickWobbleTwoA = drive.trajectoryBuilder(dropWobbleOneA.end())
                .splineToLinearHeading(new Pose2d(-28,40, Math.toRadians(180)), Math.toRadians(90))
                .build();
        telemetry.addData("PathGen:", "4/18");
        telemetry.update();

        Trajectory pickWobbleTwoACont = drive.trajectoryBuilder(pickWobbleTwoA.end())
                .lineToConstantHeading(new Vector2d(-46.5, 40))
                .build();
        telemetry.addData("PathGen:", "5/18");
        telemetry.update();

        Trajectory dropWobbleTwoA = drive.trajectoryBuilder(pickWobbleTwoACont.end())
                .lineToLinearHeading(new Pose2d(-7, 55, Math.toRadians(60)))
                .build();
        telemetry.addData("PathGen:", "6/18");
        telemetry.update();

        Trajectory parkA = drive.trajectoryBuilder(dropWobbleTwoA.end())
                .lineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "11/18");
        telemetry.update();

        Trajectory parkACont = drive.trajectoryBuilder(parkA.end())
                .lineToLinearHeading(new Pose2d(0,20, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "12/18");
        telemetry.update();

        Trajectory dropWobbleB = drive.trajectoryBuilder(highGoal.end())
                .splineToLinearHeading(new Pose2d(25, 43), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "7/18");
        telemetry.update();

        Trajectory pickWobbleTwoB = drive.trajectoryBuilder(dropWobbleB.end())
                .lineToLinearHeading(new Pose2d(-10,48, Math.toRadians(180)))
                .build();
        telemetry.addData("PathGen:", "8/18");
        telemetry.update();

        Trajectory pickWobbleTwoBCont = drive.trajectoryBuilder(pickWobbleTwoB.end())
                .lineToLinearHeading(new Pose2d(-37.7,41, Math.toRadians(215)))
                .build();
        telemetry.addData("PathGen:", "9/18");
        telemetry.update();

        Trajectory highGoalTwo = drive.trajectoryBuilder(pickWobbleTwoBCont.end())
                .lineToLinearHeading(new Pose2d(-4.5, 39, Math.toRadians(-20)))
                .build();
        telemetry.addData("PathGen:", "1/18");
        telemetry.update();

        Trajectory dropWobbleTwoB = drive.trajectoryBuilder(highGoalTwo.end())
                .lineTo(new Vector2d(13, 30))
                .build();
        telemetry.addData("PathGen:", "11/18");
        telemetry.update();

        Trajectory parkC = drive.trajectoryBuilder(dropWobbleTwoC.end())
                .lineToLinearHeading(new Pose2d(2,0))
                .build();
        telemetry.addData("PathGen:", "4/18");
        telemetry.update();

        //let drive team know its safe to start the program
        telemetry.addData(">", "street lmao");
        telemetry.update();



        while (!isStarted()) {
            telemetry.addData("Values", valOne + "   " + valTwo + "   " + valThree + "   " + valFour);
            if (valFour == 0) {
                key = 2;
                telemetry.addLine("full");
                telemetry.update();
            } else if (valThree == 0 || valOne == 0) {
                key = 1;
                telemetry.addLine("half");
                telemetry.update();
            } else {
                key = 0;
                telemetry.addLine("empty");
                telemetry.update();
            }
            telemetry.update();
        }


        waitForStart();

        if (isStopRequested()) return;

        drive.intakeBlocker.setPosition(0.04);
        drive.boomerServo.setPosition(.45);



        drive.wobbleGoalArmOne.setPosition(.8);
        drive.wobbleGoalArmTwo.setPosition(.8);

        runtime.reset();
        veloTimer.reset();
        drive.setPoseEstimate(new Pose2d(-64,43,0));



        switch (key) {

            case 0:

                drive.shooterOne.setVelocity(1310);
                drive.shooterTwo.setPower(drive.shooterOne.getPower());

                drive.followTrajectory(highGoal);

                while (drive.shooterOne.getVelocity() < 1300 && opModeIsActive()) {
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                }

                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.spindexer.setTargetPosition(-1100);
                drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.spindexer.setPower(-1);
                while (drive.spindexer.getCurrentPosition() > -1100 && opModeIsActive()) {
                    telemetry.addData("transfer", drive.spindexer.getCurrentPosition());
                    telemetry.update();
                }

                drive.spindexer.setPower(0);
                shooter.setPower(0,drive);

                drive.followTrajectory(dropWobbleOneA);

                drive.intakeBlocker.setPosition(0.3);
                drive.wobbleGoalArmOne.setPosition(.35);
                telemetry.addLine("eeeeeeeeee");
                telemetry.update();
                drive.wobbleGoalArmTwo.setPosition(.35);
                telemetry.addLine("eeeeeeeeee");
                telemetry.update();
                sleep(1500);
                drive.wobbleGoalGripper.setPosition(.3);
                drive.wobbleGoalArmOne.setPosition(.6);
                drive.wobbleGoalArmTwo.setPosition(.6);

                //goes to pick wobble
                drive.followTrajectory(pickWobbleTwoA);
                sleep(500);
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
                drive.followTrajectory(pickWobbleTwoACont);
                sleep(1000);
                drive.wobbleGoalGripper.setPosition(.65);
                sleep(1000);

                //goes to drop wobble
                drive.wobbleGoalArmOne.setPosition(.6);
                drive.wobbleGoalArmTwo.setPosition(.6);
                sleep(1500);
                drive.followTrajectory(dropWobbleTwoA);
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
                sleep(1500);
                drive.wobbleGoalGripper.setPosition(.3);
                sleep(1000);

                //park
                drive.followTrajectory(parkA);
                drive.followTrajectory(parkACont);
                drive.wobbleGoalGripper.setPosition(.65);

                PoseStorage.currentPose = drive.getPoseEstimate();
                stop();

            case 1:

                drive.shooterOne.setVelocity(1310);
                drive.shooterTwo.setPower(drive.shooterOne.getPower());

                drive.followTrajectory(highGoalBAndC);

                while (drive.shooterOne.getVelocity() < 1300 && opModeIsActive()) {
                    drive.shooterOne.setVelocity(1310);
                    drive.shooterTwo.setPower(drive.shooterOne.getPower());
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                }

                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.spindexer.setTargetPosition(-1100);
                drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.spindexer.setPower(-1);
                while (drive.spindexer.getCurrentPosition() > -1100 && opModeIsActive()) {
                    drive.shooterOne.setVelocity(1310);
                    drive.shooterTwo.setPower(drive.shooterOne.getPower());
                    telemetry.addData("transfer", drive.spindexer.getCurrentPosition());
                    telemetry.update();
                }

                drive.spindexer.setPower(0);
                shooter.setPower(0,drive);


                drive.followTrajectory(dropWobbleB);
                drive.intakeBlocker.setPosition(0.3);
                drive.wobbleGoalArmOne.setPosition(.4);
                telemetry.addLine("eeeeeeeeee");
                telemetry.update();
                drive.wobbleGoalArmTwo.setPosition(.4);
                telemetry.addLine("eeeeeeeeee");
                telemetry.update();
                sleep(1500);
                drive.wobbleGoalGripper.setPosition(.3);
                drive.wobbleGoalArmOne.setPosition(.6);
                drive.wobbleGoalArmTwo.setPosition(.6);
                drive.intake.setPower(1);

                drive.followTrajectory(pickWobbleTwoB);
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
                sleep(1500);
                drive.followTrajectory(pickWobbleTwoBCont);
                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.spindexer.setTargetPosition(-137);
                drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.spindexer.setPower(-1);
                while (drive.spindexer.getCurrentPosition() > -137 && opModeIsActive()) {}
                drive.spindexer.setPower(0);
                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.intake.setPower(0);

                drive.wobbleGoalGripper.setPosition(.65);
                sleep(1000);
                drive.wobbleGoalArmOne.setPosition(.6);
                drive.wobbleGoalArmTwo.setPosition(.6);
                sleep(1500);
                drive.intakeBlocker.setPosition(0.04);


                drive.followTrajectory(highGoalTwo);
                drive.shooterOne.setVelocity(1310);
                drive.shooterTwo.setPower(drive.shooterOne.getPower());

                while (drive.shooterOne.getVelocity() < 1300 && opModeIsActive()) {
                    drive.shooterOne.setVelocity(1310);
                    drive.shooterTwo.setPower(drive.shooterOne.getPower());
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                }

                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.spindexer.setTargetPosition(-1100);
                drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.spindexer.setPower(-1);
                while (drive.spindexer.getCurrentPosition() > -1100 && opModeIsActive()) {
                    drive.shooterOne.setVelocity(1310);
                    drive.shooterTwo.setPower(drive.shooterOne.getPower());
                    telemetry.addData("transfer", drive.spindexer.getCurrentPosition());
                    telemetry.update();
                }

                drive.spindexer.setPower(0);
                shooter.setPower(0,drive);

                drive.followTrajectory(dropWobbleTwoB);
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
                sleep(1500);
                drive.wobbleGoalGripper.setPosition(.3);

                PoseStorage.currentPose = drive.getPoseEstimate();
                stop();

            case 2:

                drive.shooterOne.setVelocity(1310);
                drive.shooterTwo.setPower(drive.shooterOne.getPower());

                drive.followTrajectory(highGoalBAndC);

                while (drive.shooterOne.getVelocity() < 1300 && opModeIsActive()) {
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                }

                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.spindexer.setTargetPosition(-1100);
                drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.spindexer.setPower(-1);
                while (drive.spindexer.getCurrentPosition() > -1100 && opModeIsActive()) {
                    telemetry.addData("transfer", drive.spindexer.getCurrentPosition());
                    telemetry.update();
                }

                drive.spindexer.setPower(0);
                shooter.setPower(0,drive);

                drive.followTrajectory(dropWobbleOneC);
                drive.intakeBlocker.setPosition(0.3);
                drive.wobbleGoalArmOne.setPosition(.4);
                telemetry.addLine("eeeeeeeeee");
                telemetry.update();
                drive.wobbleGoalArmTwo.setPosition(.4);
                telemetry.addLine("eeeeeeeeee");
                telemetry.update();
                sleep(1500);
                drive.wobbleGoalGripper.setPosition(.3);
                drive.wobbleGoalArmOne.setPosition(.6);
                drive.wobbleGoalArmTwo.setPosition(.6);

                drive.followTrajectory(pickWobbleTwoC);
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
                sleep(1500);
                drive.followTrajectory(pickWobbleTwoCCont);
                drive.wobbleGoalGripper.setPosition(.65);
                sleep(1000);
                drive.wobbleGoalArmOne.setPosition(.6);
                drive.wobbleGoalArmTwo.setPosition(.6);
                sleep(1500);

                drive.followTrajectory(dropWobbleTwoC);
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
                sleep(1500);
                drive.wobbleGoalGripper.setPosition(.3);

                drive.followTrajectory(parkC);

                PoseStorage.currentPose = drive.getPoseEstimate();
                stop();

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
            Imgproc.threshold(yCbCrChan2Mat, thresholdMat, 90, 255, Imgproc.THRESH_BINARY_INV);

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
