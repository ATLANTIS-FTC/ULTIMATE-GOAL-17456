package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.exception.MathParseException;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
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
public class SixtyFivePointAuto extends LinearOpMode {

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

    int key = -1;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter();
        ElapsedTime runtime = new ElapsedTime();

        drive.spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER, drive);

        drive.spindexer.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.FRONT, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new StageSwitchingPipeline());
        phoneCam.startStreaming(rows, cols, OpenCvCameraRotation.UPRIGHT);

        // generate power shot paths


        drive.boomerServo.setPosition(.325);
        drive.intakeBlocker.setPosition(.35);

        Trajectory powerShotA = drive.trajectoryBuilder(new Pose2d(-60,35), Math.toRadians(0))
                .lineToLinearHeading(new Pose2d(16.4, 38, Math.toRadians(5)))
                .build();
        telemetry.addData("PathGen:", "1/18");
        telemetry.update();

        Trajectory powerShotBandC = drive.trajectoryBuilder(new Pose2d(-60,35), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-35, 59), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(9,59), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(16.4, 38), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "2/18");
        telemetry.update();

        // generate wobble paths

        Trajectory dropWobbleOneA = drive.trajectoryBuilder(powerShotA.end())
                .lineToLinearHeading(new Pose2d(17, 48, Math.toRadians(90)))
                .build();
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();

        Trajectory pickWobbleTwoA = drive.trajectoryBuilder(dropWobbleOneA.end())
                .splineToLinearHeading(new Pose2d(-30,25, Math.toRadians(180)), Math.toRadians(90))
                .build();
        telemetry.addData("PathGen:", "4/18");
        telemetry.update();

        Trajectory pickWobbleTwoACont = drive.trajectoryBuilder(pickWobbleTwoA.end())
                .lineToConstantHeading(new Vector2d(-32, 3))
                .build();
        telemetry.addData("PathGen:", "5/18");
        telemetry.update();

        Trajectory dropWobbleTwoA = drive.trajectoryBuilder(pickWobbleTwoACont.end())
                .lineToLinearHeading(new Pose2d(-7, 63, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "6/18");
        telemetry.update();

        Trajectory dropWobbleB = drive.trajectoryBuilder(powerShotBandC.end())
                .splineToLinearHeading(new Pose2d(35, 35), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "7/18");
        telemetry.update();

        Trajectory pickWobbleTwoB = drive.trajectoryBuilder(dropWobbleB.end())
                .lineToLinearHeading(new Pose2d(-33,35, Math.toRadians(180)))
                .build();
        telemetry.addData("PathGen:", "8/18");
        telemetry.update();

        Trajectory pickWobbleTwoBCont = drive.trajectoryBuilder(pickWobbleTwoB.end())
                .lineToLinearHeading(new Pose2d(-29.5,6.3, Math.toRadians(180)))
                .build();
        telemetry.addData("PathGen:", "9/18");
        telemetry.update();

        Trajectory dropWobbleTwoB = drive.trajectoryBuilder(pickWobbleTwoBCont.end())
                .lineToLinearHeading(new Pose2d(21, 38, Math.toRadians(0)))
                .addDisplacementMarker(() -> {
                  sleep(7000);
                })
                .build();
        telemetry.addData("PathGen:", "10/18");
        telemetry.update();

        //generate park paths

        Trajectory parkA = drive.trajectoryBuilder(dropWobbleTwoA.end())
                .lineToLinearHeading(new Pose2d(-20, 35, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "11/18");
        telemetry.update();

        Trajectory parkACont = drive.trajectoryBuilder(parkA.end())
                .lineToLinearHeading(new Pose2d(11,27, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "12/18");
        telemetry.update();

        /*Trajectory parkB = drive.trajectoryBuilder(dropWobbleTwoB.end())
                .lineToLinearHeading(new Pose2d(-20, 35, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "6/7");
        telemetry.update();

        Trajectory parkBCont = drive.trajectoryBuilder(parkB.end())
                .lineToLinearHeading(new Pose2d(15, 35, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "7/7");
        telemetry.update();*/

        Trajectory dropWobbleC = drive.trajectoryBuilder(powerShotBandC.end())
                .splineToConstantHeading(new Vector2d(53,36), Math.toRadians(90))
                .build();
        telemetry.addData("PathGen:", "13/18");
        telemetry.update();
        //generate park paths

        Trajectory pickWobbleTwoC = drive.trajectoryBuilder(dropWobbleC.end())
                .splineToLinearHeading(new Pose2d(-39, 40, Math.toRadians(180)), Math.toRadians(90))
                .build();
        telemetry.addData("PathGen:", "14/18");
        telemetry.update();

//        Trajectory pickWobbleTwoCCont = drive.trajectoryBuilder(pickWobbleTwoC.end())
//                .lineToLinearHeading(new Pose2d(-32.3,15, Math.toRadians(180)))
//                .build();
//        telemetry.addData("PathGen:", "5/7");
//        telemetry.update();

        Trajectory pickWobbleTwoCCont2 = drive.trajectoryBuilder(pickWobbleTwoC.end())
                .lineToLinearHeading(new Pose2d(-39,11, Math.toRadians(180)))
                .build();
        telemetry.addData("PathGen:", "15/18");
        telemetry.update();

        Trajectory dropWobbleTwoC = drive.trajectoryBuilder(pickWobbleTwoCCont2.end())
                .splineToLinearHeading(new Pose2d(0,10, Math.toRadians(0)), Math.toRadians(180))
                .build();
        telemetry.addData("PathGen", "16/18");
        telemetry.update();

        Trajectory dropWobbleTwoCCont = drive.trajectoryBuilder(dropWobbleTwoC.end())
                .splineToConstantHeading(new Vector2d(44.5,55), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen", "17/18");
        telemetry.update();

        Trajectory parkC = drive.trajectoryBuilder(dropWobbleTwoC.end())
                .lineTo(new Vector2d(12, 39))
                .build();
        telemetry.addData("PathGen", "18/18");
        telemetry.update();

//
//        Trajectory pickWobbleTwoACont2 = drive.trajectoryBuilder(pickWobbleTwoACont.end())
//                .splineToConstantHeading(new Vector2d(-37, 0), Math.toRadians(90))
//                .build();
//        telemetry.addData("PathGen:", "7/7");
//        telemetry.update();



        //let drive team know its safe to start the program
        telemetry.addData(">", "It is what it is");
        telemetry.update();

        sleep(2000);

        if (valFour == 0) {
            key = 2;
            telemetry.addLine("full");
        } else if (valOne == 0 || valTwo == 0) {
            key = 1;
            telemetry.addLine("half");
        } else {
            key = 0;
            telemetry.addLine("empty");
        }

        telemetry.update();

        waitForStart();

        runtime.reset();
        while (opModeIsActive()) {
            telemetry.addData("Values", valOne + "   " + valTwo + "   " + valThree + "   " + valFour);
            telemetry.addData("Height", rows);
            telemetry.addData("Width", cols);

            telemetry.update();

            drive.setPoseEstimate(new Pose2d(-60,35,0));


            //drive.ringGate.setPosition(.3);

            if (isStopRequested()) return;

            shooter.setPower(-1, drive);

            drive.wobbleGoalArmOne.setPosition(.6);
            drive.wobbleGoalArmTwo.setPosition(.6);
            drive.shooterOne.setPower(-1);
            drive.shooterTwo.setPower(-1);


            switch (key) {

                case 0: // empty stack

                    //drives up
                    drive.followTrajectory(powerShotA);

                    //open ring gate
                    drive.ringGate.setPosition(0);

                    //waits for wheel to spin up to 3600 rpm
                    while (drive.shooterOne.getVelocity() / 28 * 60 > -4100 && opModeIsActive()){
                        telemetry.addData("Shooter RPM:", drive.shooterOne.getVelocity() / 28 * 60);
                        telemetry.update();
                    }

                    //runs spindexer to shoot first ring
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(260);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    drive.spindexer.setPower(-1);
                    while (Math.abs(drive.spindexer.getCurrentPosition()) < 260 && opModeIsActive()) {
                        telemetry.addLine("first shot");
                        telemetry.update();
                    }
                    drive.spindexer.setPower(0);
                    sleep(1000);

                    drive.turn(Math.toRadians(-4));

                    //waits for wheel to spin up
                    while (drive.shooterOne.getVelocity() / 28 * 60 > -4100 && opModeIsActive()){
                        telemetry.addData("Shooter RPM:", drive.shooterOne.getVelocity() / 28 * 60);
                        telemetry.update();
                    }

                    //runs spindexer to shoot second ring
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(1000);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    drive.spindexer.setPower(-1);
                    while (Math.abs(drive.spindexer.getCurrentPosition()) < 1000 && opModeIsActive()) {
                        telemetry.addLine("first shot");
                        telemetry.update();
                    }
                    drive.spindexer.setPower(0);
                    sleep(1000);

                    //wait for wheel to spin up
                    /*while (drive.shooterOne.getVelocity() / 28 * 60 > -3600 && opModeIsActive()){
                        telemetry.addData("Shooter RPM:", drive.shooterOne.getVelocity() / 28 * 60);
                        telemetry.update();
                    }


                    //runs spindexer to shoot first ring
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(400);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    drive.spindexer.setPower(-1);
                    while (Math.abs(drive.spindexer.getCurrentPosition()) < 400 && opModeIsActive()) {
                        telemetry.addLine("first shot");
                        telemetry.update();
                    }
                    drive.spindexer.setPower(0);
                    sleep(1000);*/

                    //goes to drop wobble
                    drive.followTrajectory(dropWobbleOneA);
                    drive.shooterOne.setPower(0);
                    drive.shooterTwo.setPower(0);
                    drive.wobbleGoalArmOne.setPosition(.2);
                    drive.wobbleGoalArmTwo.setPosition(.2);
                    sleep(1500);
                    drive.wobbleGoalArmOne.setPosition(.35);
                    drive.wobbleGoalArmTwo.setPosition(.35);

                    //goes to pick wobble
                    drive.followTrajectory(pickWobbleTwoA);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);
                    drive.wobbleGoalGripper.setPosition(.3);
                    drive.followTrajectory(pickWobbleTwoACont);
                    drive.wobbleGoalGripper.setPosition(.65);
                    sleep(1000);
                    drive.wobbleGoalArmOne.setPosition(.4);
                    drive.wobbleGoalArmTwo.setPosition(.4);
                    sleep(1500);

                    //goes to drop wobble
                    drive.followTrajectory(dropWobbleTwoA);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);
                    sleep(1500);
                    drive.wobbleGoalGripper.setPosition(.3);
                    sleep(1000);

                    //park
                    drive.followTrajectory(parkA);
                    drive.followTrajectory(parkACont);


                    //either is not able to pick up wobble goal or smh drops it by accident the grip is kinda off but works in teleop
//                    drive.followTrajectory(pickWobbleTwoACont2);
//                    drive.wobbleGoalArmOne.setPosition(0);
//                    drive.wobbleGoalArmTwo.setPosition(0);
//                    drive.wobbleGoalGripper.setPosition(.3);



//                    drive.followTrajectory(pickWobbleTwoACont);

                    break;

                case 1: // half stack

                    drive.followTrajectory(powerShotBandC);

                    //open ring gate
                    drive.ringGate.setPosition(0);
                    drive.turn(Math.toRadians(1));

                    //waits for wheel to spin up to 3600 rpm
                    while (drive.shooterOne.getVelocity() / 28 * 60 > -4100 && opModeIsActive()){
                        telemetry.addData("Shooter RPM:", drive.shooterOne.getVelocity() / 28 * 60);
                        telemetry.update();
                    }

                    //runs spindexer to shoot first ring
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(250);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    drive.spindexer.setPower(-1);
                    while (Math.abs(drive.spindexer.getCurrentPosition()) < 250 && opModeIsActive()) {
                        telemetry.addLine("first shot");
                        telemetry.update();
                    }
                    drive.spindexer.setPower(0);

//                    drive.turn(Math.toRadians(-10));

                    //waits for wheel to spin up
                    while (drive.shooterOne.getVelocity() / 28 * 60 > -4100 && opModeIsActive()){
                        telemetry.addData("Shooter RPM:", drive.shooterOne.getVelocity() / 28 * 60);
                        telemetry.update();
                    }

                    //runs spindexer to shoot second ring
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(1000);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    drive.spindexer.setPower(-1);
                    while (Math.abs(drive.spindexer.getCurrentPosition()) < 1000 && opModeIsActive()) {
                        telemetry.addLine("first shot");
                        telemetry.update();
                    }
                    drive.spindexer.setPower(0);
                    shooter.setPower(0, drive);

                    //goes to drop wobble
                    drive.followTrajectory(dropWobbleB);
                    drive.ringGate.setPosition(.35);
                    drive.shooterOne.setPower(0);
                    drive.shooterTwo.setPower(0);
                    drive.wobbleGoalArmOne.setPosition(.1);
                    drive.wobbleGoalArmTwo.setPosition(.1);
                    sleep(1500);
                    drive.wobbleGoalArmOne.setPosition(25);
                    drive.wobbleGoalArmTwo.setPosition(.25);
                    drive.wobbleGoalGripper.setPosition(.3);
                    sleep(1000);

                    //goes to pick up wobble
                    drive.followTrajectory(pickWobbleTwoB);
                    drive.wobbleGoalGripper.setPosition(.3);
                    drive.followTrajectory(pickWobbleTwoBCont);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);
                    sleep(1500);
                    drive.wobbleGoalGripper.setPosition(.65);
                    sleep(1000);
                    drive.wobbleGoalArmOne.setPosition(.4);
                    drive.wobbleGoalArmTwo.setPosition(.4);
                    sleep(1500);

                    //goes to drop wobble
                    drive.followTrajectory(dropWobbleTwoB);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);
                    sleep(1500);
                    drive.wobbleGoalGripper.setPosition(0);
                    sleep(1000);
//
//                    drive.followTrajectory(parkB);
//                    drive.followTrajectory(parkBCont);

                    drive.followTrajectory(dropWobbleB);
                    drive.shooterOne.setPower(0);
                    drive.shooterTwo.setPower(0);
                    drive.wobbleGoalArmOne.setPosition(.15);
                    drive.wobbleGoalArmTwo.setPosition(.15);
                    sleep(1500);
                    drive.wobbleGoalArmOne.setPosition(.3);
                    drive.wobbleGoalArmTwo.setPosition(.3);

                    //goes to pick wobble
                    drive.followTrajectory(pickWobbleTwoB);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);
                    drive.wobbleGoalGripper.setPosition(.3);
                    drive.followTrajectory(pickWobbleTwoBCont);
                    drive.wobbleGoalGripper.setPosition(.65);
                    sleep(1000);
                    drive.wobbleGoalArmOne.setPosition(.4);
                    drive.wobbleGoalArmTwo.setPosition(.4);
                    sleep(1500);

                    //goes to drop wobble
                    drive.followTrajectory(dropWobbleTwoB);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);

                    sleep(5000);
                    drive.wobbleGoalGripper.setPosition(.3);


                    break;

                case 2: // full stack

                    drive.followTrajectory(powerShotBandC);

                    //open ring gate
                    drive.ringGate.setPosition(0);
                    drive.turn(Math.toRadians(1));

                    //waits for wheel to spin up to 3600 rpm
                    while (drive.shooterOne.getVelocity() / 28 * 60 > -4100 && opModeIsActive()){
                        telemetry.addData("Shooter RPM:", drive.shooterOne.getVelocity() / 28 * 60);
                        telemetry.update();
                    }

                    //runs spindexer to shoot first ring
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(250);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    drive.spindexer.setPower(-1);
                    while (Math.abs(drive.spindexer.getCurrentPosition()) < 250 && opModeIsActive()) {
                        telemetry.addLine("first shot");
                        telemetry.update();
                    }
                    drive.spindexer.setPower(0);

//                    drive.turn(Math.toRadians(-11));
//
//                    //waits for wheel to spin up
                    while (drive.shooterOne.getVelocity() / 28 * 60 > -4100 && opModeIsActive()){
                        telemetry.addData("Shooter RPM:", drive.shooterOne.getVelocity() / 28 * 60);
                        telemetry.update();
                    }

                    //runs spindexer to shoot second ring
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(1000);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    drive.spindexer.setPower(-1);
                    while (Math.abs(drive.spindexer.getCurrentPosition()) < 1000 && opModeIsActive()) {
                        telemetry.addLine("first shot");
                        telemetry.update();
                    }
                    drive.spindexer.setPower(0);
                    drive.shooterOne.setPower(0);
                    drive.shooterTwo.setPower(0);
                    drive.followTrajectory(dropWobbleC);

                    drive.wobbleGoalArmOne.setPosition(.2);
                    drive.wobbleGoalArmTwo.setPosition(.2);
                    drive.wobbleGoalArmOne.setPosition(.3);
                    drive.wobbleGoalArmTwo.setPosition(.3);
                    sleep(1500);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);
                    drive.wobbleGoalArmOne.setPosition(.3);
                    drive.wobbleGoalArmTwo.setPosition(.3);
//                    drive.turn(Math.toRadians(-180));
                    drive.followTrajectory(pickWobbleTwoC);
                    drive.wobbleGoalArmOne.setPosition(0);
                    drive.wobbleGoalArmTwo.setPosition(0);
                    sleep(1500);
                    drive.wobbleGoalGripper.setPosition(.3);
                    sleep(1000);
//                    drive.followTrajectory(pickWobbleTwoCCont);

                    drive.followTrajectory(pickWobbleTwoCCont2);
                    drive.wobbleGoalGripper.setPosition(.65);
                    sleep(1000);
                    drive.wobbleGoalArmOne.setPosition(.4);
                    drive.wobbleGoalArmTwo.setPosition(.4);
                    sleep(1500);
                    drive.followTrajectory(dropWobbleTwoC);
                    drive.followTrajectory(dropWobbleTwoCCont);
//                    drive.turn(Math.toRadians(-180));
                    drive.wobbleGoalArmOne.setPosition(0.2);
                    drive.wobbleGoalArmTwo.setPosition(0.2);
                    sleep(1000);
                    drive.wobbleGoalGripper.setPosition(.3);
                    drive.wobbleGoalArmOne.setPosition(.4);
                    drive.wobbleGoalArmTwo.setPosition(.4);

                    drive.followTrajectory(parkC);

                    break;

            }

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

