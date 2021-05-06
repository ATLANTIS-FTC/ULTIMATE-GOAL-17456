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

@Autonomous(group = "Feb")
public class FebAuto extends LinearOpMode {


    int key = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter();
        ElapsedTime runtime = new ElapsedTime();

        ElapsedTime veloTimer = new ElapsedTime();

        shooter.lastTargetVelo = shooter.targetVelo;

        double motorPos = drive.shooterOne.getCurrentPosition();
        double motorVelo = drive.shooterOne.getVelocity();

        drive.shooterOne.setDirection(DcMotorSimple.Direction.REVERSE);
        drive.shooterTwo.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.intakeBlocker.setPosition(0.0125);
        drive.boomerServo.setPosition(.45);
        shooter.targetVelo = -1369.9;

        shooter.veloController.setTargetVelocity(shooter.targetVelo);
        shooter.veloController.setTargetAcceleration((shooter.targetVelo-shooter.lastTargetVelo) / veloTimer.seconds());
        double power = shooter.veloController.update(motorPos, motorVelo);

        Trajectory powerShotA = drive.trajectoryBuilder(new Pose2d(-64,43), Math.toRadians(0))
                .splineTo(new Vector2d(-4, 56), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "1/18");
        telemetry.update();

        Trajectory powerShotBandC = drive.trajectoryBuilder(new Pose2d(-64,43), Math.toRadians(0))
                .splineTo(new Vector2d(-17, 61), Math.toRadians(0))
                .splineTo(new Vector2d(-4,56), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "1/18");
        telemetry.update();

        Trajectory dropWobbleOneA = drive.trajectoryBuilder(powerShotA.end())
                .lineToLinearHeading(new Pose2d(8, 63, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();

        Trajectory pickWobbleTwoA = drive.trajectoryBuilder(dropWobbleOneA.end())
                .splineToLinearHeading(new Pose2d(-32,20, Math.toRadians(180)), Math.toRadians(90))
                .build();
        telemetry.addData("PathGen:", "4/18");
        telemetry.update();

        Trajectory pickWobbleTwoACont = drive.trajectoryBuilder(pickWobbleTwoA.end())
                .lineToConstantHeading(new Vector2d(-35, 15))
                .build();
        telemetry.addData("PathGen:", "5/18");
        telemetry.update();

        Trajectory dropWobbleTwoA = drive.trajectoryBuilder(pickWobbleTwoACont.end())
                .lineToLinearHeading(new Pose2d(1, 67.3, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "6/18");
        telemetry.update();

        Trajectory parkA = drive.trajectoryBuilder(dropWobbleTwoA.end())
                .lineToLinearHeading(new Pose2d(-20, 35, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "11/18");
        telemetry.update();

        Trajectory parkACont = drive.trajectoryBuilder(parkA.end())
                .lineToLinearHeading(new Pose2d(20,20, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "12/18");
        telemetry.update();

        Trajectory dropWobbleB = drive.trajectoryBuilder(powerShotBandC.end())
                .splineToLinearHeading(new Pose2d(35, 35), Math.toRadians(0))
                .build();
        telemetry.addData("PathGen:", "7/18");
        telemetry.update();

        Trajectory pickWobbleTwoB = drive.trajectoryBuilder(dropWobbleB.end())
                .splineToLinearHeading(new Pose2d(-27,55, Math.toRadians(180)), Math.toRadians(-90))
                .build();
        telemetry.addData("PathGen:", "8/18");
        telemetry.update();

        Trajectory pickWobbleTwoBCont = drive.trajectoryBuilder(pickWobbleTwoB.end())
                .lineToLinearHeading(new Pose2d(-32,14, Math.toRadians(180)))
                .build();
        telemetry.addData("PathGen:", "9/18");
        telemetry.update();

        Trajectory pickWobbleTwoBContt = drive.trajectoryBuilder(pickWobbleTwoBCont.end())
                .lineToLinearHeading(new Pose2d(-35,14, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "10/18");
        telemetry.update();

        Trajectory dropWobbleTwoB = drive.trajectoryBuilder(pickWobbleTwoBContt.end())
                .lineToLinearHeading(new Pose2d(25, 35, Math.toRadians(0)))
                .build();
        telemetry.addData("PathGen:", "11/18");
        telemetry.update();

        //let drive team know its safe to start the program
        telemetry.addData(">", "street lmao");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;


        runtime.reset();
        veloTimer.reset();
        drive.setPoseEstimate(new Pose2d(-64,43,0));
        shooter.setPower(power, drive);
        drive.followTrajectory(powerShotA);

        drive.wobbleGoalArmOne.setPosition(.6);
        drive.wobbleGoalArmTwo.setPosition(.6);
        drive.shooterOne.setPower(-1);
        drive.shooterTwo.setPower(-1);



        switch (key) {

            case 0:

                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.spindexer.setTargetPosition(-1100);
                drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                drive.spindexer.setPower(-1);
                while (drive.spindexer.getCurrentPosition() > -1100 && opModeIsActive()) {}
                drive.spindexer.setPower(0);

                sleep(700);

                drive.followTrajectory(dropWobbleOneA);
                drive.shooterOne.setPower(0);
                drive.shooterTwo.setPower(0);
                drive.turn(Math.toRadians(60));
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

                //goes to drop wobble
                drive.wobbleGoalArmOne.setPosition(.4);
                drive.wobbleGoalArmTwo.setPosition(.4);
                sleep(1500);
                drive.followTrajectory(dropWobbleTwoA);
                drive.turn(Math.toRadians(Math.toRadians(50)));
                sleep(1500);
                drive.wobbleGoalArmOne.setPosition(0);
                drive.wobbleGoalArmTwo.setPosition(0);
                sleep(1500);
                drive.wobbleGoalGripper.setPosition(.3);
                sleep(1000);

                //park
                drive.followTrajectory(parkA);
                drive.followTrajectory(parkACont);

                stop();

            case 1:

                drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                drive.spindexer.setTargetPosition(-1100);
                drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                runtime.reset();
                drive.spindexer.setPower(-1);
                while (drive.spindexer.getCurrentPosition() > -1100 && opModeIsActive()) {}
                drive.spindexer.setPower(0);

                drive.followTrajectory(dropWobbleB);
                drive.shooterOne.setPower(0);
                drive.shooterTwo.setPower(0);
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
                sleep(1500);
                drive.wobbleGoalArmOne.setPosition(.35);
                drive.wobbleGoalArmTwo.setPosition(.35);

                //goes to pick wobble
                drive.followTrajectory(pickWobbleTwoB);
                drive.wobbleGoalArmOne.setPosition(0);
                drive.wobbleGoalArmTwo.setPosition(0);
                drive.followTrajectory(pickWobbleTwoBCont);
                drive.wobbleGoalGripper.setPosition(.3);
                drive.followTrajectory(pickWobbleTwoBContt);
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
                drive.wobbleGoalGripper.setPosition(.3);
                sleep(1000);
           }

//        //drive.turn(Math.toRadians(-5));
//
////        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.spindexer.setTargetPosition(-180);
//        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        runtime.reset();
//        drive.spindexer.setPower(-1);
//        while (drive.spindexer.getCurrentPosition() > -160 && opModeIsActive()) {}
//        drive.spindexer.setPower(0);
//
//        drive.turn(Math.toRadians(-7.4));
//
////        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.spindexer.setTargetPosition(-90);
//        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.spindexer.setPower(-1);
//        while (drive.spindexer.getCurrentPosition() > -90 && opModeIsActive()) {}
//        drive.spindexer.setPower(0);
//
//        drive.turn(Math.toRadians(-7.5));
//
////        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        drive.spindexer.setTargetPosition(-300);
//        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        drive.spindexer.setPower(-1);
//        while (drive.spindexer.getCurrentPosition() > -300 && opModeIsActive()) {}
//        drive.spindexer.setPower(0);*/


    }
}
