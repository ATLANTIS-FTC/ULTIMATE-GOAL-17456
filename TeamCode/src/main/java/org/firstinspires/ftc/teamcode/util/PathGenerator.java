package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;

import java.util.ArrayList;

public class PathGenerator {

    SampleMecanumDrive drive;
    Telemetry telemetry;

    public ArrayList<Trajectory> paths = new ArrayList<Trajectory>();

    public PathGenerator(SampleMecanumDrive drive, Telemetry telemetry) {
        this.drive = drive;
        this.telemetry = telemetry;
    }

    public void buildPaths() {
        Trajectory highGoal = drive.trajectoryBuilder(new Pose2d(-64,43), Math.toRadians(0))
                .lineTo(new Vector2d(-7, 43))
                .build();
        paths.add(highGoal);
        telemetry.addData("PathGen:", "1/18");
        telemetry.update();

        Trajectory dropWobbleOneA = drive.trajectoryBuilder(highGoal.end())
                .lineToLinearHeading(new Pose2d(-2, 59, Math.toRadians(60)))
                .build();
        paths.add(dropWobbleOneA);
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();

        Trajectory dropWobbleOneC = drive.trajectoryBuilder(highGoal.end())
                .lineToLinearHeading(new Pose2d(55, 75, Math.toRadians(45)))
                .build();
        paths.add(dropWobbleOneC);
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();

        Trajectory pickWobbleTwoC = drive.trajectoryBuilder(dropWobbleOneC.end())
                .lineToLinearHeading(new Pose2d(-41,43, Math.toRadians(-90)))
                .build();
        paths.add(pickWobbleTwoC);
        telemetry.addData("PathGen:", "8/18");
        telemetry.update();

        Trajectory pickWobbleTwoCCont = drive.trajectoryBuilder(pickWobbleTwoC.end())
                .lineToLinearHeading(new Pose2d(-42,35, Math.toRadians(-90)))
                .build();
        paths.add(pickWobbleTwoCCont);
        telemetry.addData("PathGen:", "9/18");
        telemetry.update();

        Trajectory dropWobbleTwoC = drive.trajectoryBuilder(pickWobbleTwoCCont.end())
                .lineToLinearHeading(new Pose2d(40, 75, Math.toRadians(0)))
                .build();
        paths.add(dropWobbleTwoC);
        telemetry.addData("PathGen:", "3/18");
        telemetry.update();


        Trajectory pickWobbleTwoA = drive.trajectoryBuilder(dropWobbleOneA.end())
                .splineToLinearHeading(new Pose2d(-31,17.8, Math.toRadians(180)), Math.toRadians(90))
                .build();
        paths.add(pickWobbleTwoA);
        telemetry.addData("PathGen:", "4/18");
        telemetry.update();

        Trajectory pickWobbleTwoACont = drive.trajectoryBuilder(pickWobbleTwoA.end())
                .lineToConstantHeading(new Vector2d(-39, 17.8))
                .build();
        paths.add(pickWobbleTwoACont);
        telemetry.addData("PathGen:", "5/18");
        telemetry.update();

        Trajectory dropWobbleTwoA = drive.trajectoryBuilder(pickWobbleTwoACont.end())
                .splineToLinearHeading(new Pose2d(-7, 70), Math.toRadians(60))
                .build();
        paths.add(dropWobbleTwoA);
        telemetry.addData("PathGen:", "6/18");
        telemetry.update();

        Trajectory parkA = drive.trajectoryBuilder(dropWobbleTwoA.end())
                .lineToLinearHeading(new Pose2d(-20, 20, Math.toRadians(0)))
                .build();
        paths.add(parkA);
        telemetry.addData("PathGen:", "11/18");
        telemetry.update();

        Trajectory parkACont = drive.trajectoryBuilder(parkA.end())
                .lineToLinearHeading(new Pose2d(12,20, Math.toRadians(0)))
                .build();
        paths.add(parkACont);
        telemetry.addData("PathGen:", "12/18");
        telemetry.update();

        Trajectory dropWobbleB = drive.trajectoryBuilder(highGoal.end())
                .splineToLinearHeading(new Pose2d(35, 52), Math.toRadians(0))
                .build();
        paths.add(dropWobbleB);
        telemetry.addData("PathGen:", "7/18");
        telemetry.update();

        Trajectory pickWobbleTwoB = drive.trajectoryBuilder(dropWobbleB.end())
                .lineToLinearHeading(new Pose2d(-40.51,42, Math.toRadians(-90)))
                .build();
        paths.add(pickWobbleTwoB);
        telemetry.addData("PathGen:", "8/18");
        telemetry.update();

        Trajectory pickWobbleTwoBCont = drive.trajectoryBuilder(pickWobbleTwoB.end())
                .lineToLinearHeading(new Pose2d(-41.4,35, Math.toRadians(-90)))
                .build();
        paths.add(pickWobbleTwoBCont);
        telemetry.addData("PathGen:", "9/18");
        telemetry.update();

        Trajectory dropWobbleTwoB = drive.trajectoryBuilder(pickWobbleTwoBCont.end())
                .lineToLinearHeading(new Pose2d(20, 54, Math.toRadians(0)))
                .build();
        paths.add(dropWobbleTwoB);
        telemetry.addData("PathGen:", "11/18");
        telemetry.update();

        Trajectory parkC = drive.trajectoryBuilder(dropWobbleTwoC.end())
                .lineToLinearHeading(new Pose2d(5,68))
                .build();
        paths.add(parkC);
        telemetry.addData("PathGen:", "4/18");
        telemetry.update();

        //let drive team know its safe to start the program
        telemetry.addData(">", "street lmao");
        telemetry.update();
    }
}
