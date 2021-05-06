package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;

@Config
@TeleOp(name="Two Motor Test", group = "teleop")
public class TwoMotorTest extends LinearOpMode {
    DcMotorEx motor1, motor2;

    public static double kP = 700;
    public static double kD = 50;
    public static double kF = 10;

    // for reference, mine are kP 450, kD 4, kF 12

    public static double targetVelo = 1000;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime timer = new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        motor1 = hardwareMap.get(DcMotorEx.class, "s1");
        motor2 = hardwareMap.get(DcMotorEx.class, "s2");

        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.FORWARD);

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoeffs = new PIDFCoefficients(kP, 0, kD, kF);

        motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);



        waitForStart();
        while (!isStopRequested()) {
            motor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, 0, kD, kF));

            if(gamepad1.a) {
                motor1.setVelocity(targetVelo);
                motor2.setPower(motor1.getPower());
            } else if (gamepad1.b) {
                motor1.setVelocity(0);
                motor2.setPower(motor1.getPower());
            }

            telemetry.addData("Target", targetVelo);
            telemetry.addData("Motor 1 Velocity", motor1.getVelocity());

            telemetry.update();
        }
    }
}