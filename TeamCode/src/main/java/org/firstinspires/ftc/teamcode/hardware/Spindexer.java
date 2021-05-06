package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.tuning.TuningController;

import TeleOp.NovaOp;

public class Spindexer {


    public void ringPositionOne(SampleMecanumDrive drive) {
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.spindexer.setTargetPosition(-132);
        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.spindexer.setPower(-1);
        while (drive.spindexer.getCurrentPosition() > -132) {}

        drive.spindexer.setPower(0);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ringPositionTwo(SampleMecanumDrive drive){
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.spindexer.setTargetPosition(-60);
        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.spindexer.setPower(-1);
        while (drive.spindexer.getCurrentPosition() > -40) {}

        drive.spindexer.setPower(0);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ringPositionThree(SampleMecanumDrive drive) {
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.spindexer.setTargetPosition(-170);
        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.spindexer.setPower(-1);
        while (drive.spindexer.getCurrentPosition() > -170) {}

        drive.spindexer.setPower(0);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ringPositionFour(SampleMecanumDrive drive) {
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.spindexer.setTargetPosition(-100);
        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.spindexer.setPower(-1);
        while (drive.spindexer.getCurrentPosition() > -100) {}

        drive.spindexer.setPower(0);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void ringPositionFive(SampleMecanumDrive drive) {
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.spindexer.setTargetPosition(-142);
        drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        drive.spindexer.setPower(-1);
        while (drive.spindexer.getCurrentPosition() > -142) {}

        drive.spindexer.setPower(0);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}