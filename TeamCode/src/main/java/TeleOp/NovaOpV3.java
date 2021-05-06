package TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Shooter;
import org.firstinspires.ftc.teamcode.hardware.Spindexer;
import org.firstinspires.ftc.teamcode.tuning.TuningController;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
@Config
@TeleOp(group = "drive")
public class NovaOpV3 extends LinearOpMode {

    // goal coords: 70, 36

    long loopCount;
    double shooterPosition;
    double deltaRotations;

    public static double DRAWING_TARGET_RADIUS = 2;

    // Define 2 states, driver control or alignment control
    enum Mode {
        NORMAL_CONTROL,
        ALIGN_TO_POINT,
        POWER_SHOTS
    }

    private Mode currentMode = Mode.NORMAL_CONTROL;

    private PIDFController headingController = new PIDFController(SampleMecanumDrive.HEADING_PID);

    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    private Vector2d targetPosition = new Vector2d(70, 40);

    ElapsedTime veloTimer = new ElapsedTime();
    ElapsedTime veloTimerPwr = new ElapsedTime();
    ElapsedTime shooterTimer = new ElapsedTime();
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    double kP = 700;
    double kD = 50;
    double kF = 10;


    @Override
    public void runOpMode() throws InterruptedException {



        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Shooter shooter = new Shooter();
        Spindexer spindexer = new Spindexer();

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, drive);
        shooter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT, drive);
        drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.spindexer.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        boolean arm = false;
        boolean grip = false;
        boolean autoAim = false;
        boolean autoshoot = false;
        boolean intake = false;
        boolean spindex = false;
        double counter = 0;

        // We want to turn off velocity control for teleop
        // Velocity control per wheel is not necessary outside of motion profiled auto
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.getLocalizer().setPoseEstimate(PoseStorage.currentPose);


        waitForStart();
        if (isStopRequested()) return;

        drive.ringGate.setPosition(0);

        drive.boomerServo.setPosition(.45);
        veloTimer.reset();
        veloTimerPwr.reset();
        shooterTimer.reset();

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());




        while (!isStopRequested() && opModeIsActive()) {
            Pose2d poseEstimate = drive.getLocalizer().getPoseEstimate();

            drive.update();
            // Declare a drive direction
            // Pose representing desired x, y, and angular velocity
            Pose2d driveDirection = new Pose2d();
            Pose2d myPose = drive.getPoseEstimate();



            telemetry.addData("mode", currentMode);

            // Declare telemetry packet for dashboard field drawing
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();



            switch (currentMode) {
                case NORMAL_CONTROL:
                    // Switch into alignment mode if `a` is pressed
                    if (gamepad1.x) {
                        currentMode = Mode.ALIGN_TO_POINT;
                    }

                    if (gamepad1.dpad_up) {
                        currentMode = Mode.POWER_SHOTS;
                    }

                    // telemetry
                    shooter.currentRPM = drive.shooterOne.getVelocity() / 28 * 60;
                    shooter.wheelRPM = shooter.currentRPM * 1.37;
                    telemetry.addData("x", poseEstimate.getX());
                    telemetry.addData("y", poseEstimate.getY());
                    telemetry.addData("hea" +
                            "ding", poseEstimate.getHeading());
                    telemetry.addData("Shooter RPM:", shooter.currentRPM);
                    telemetry.addData("Wheel RPM:", shooter.wheelRPM);
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.addData("transfer", drive.spindexer.getCurrentPosition());
                    telemetry.addData("arm", arm);

                    telemetry.update();

                    shooter.setPower(gamepad1.right_trigger, drive);

                    // Standard teleop control
                    // Convert gamepad input into desired pose velocity
                    driveDirection = new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    );
                    break;
                case ALIGN_TO_POINT:
//                    // Switch back into normal driver control mode if `b` is pressed
//                    Trajectory autoShoot = drive.trajectoryBuilder(myPose)
//                            .lineToLinearHeading(new Pose2d(0, 45, Math.toRadians(0)))
//                            .build();

                    if (gamepad1.x) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    if (gamepad1.dpad_up) {
                        currentMode = Mode.POWER_SHOTS;
                    }


                    drive.intake.setPower(0);
                    drive.spindexer.setPower(0);

                    drive.shooterOne.setDirection(DcMotorSimple.Direction.FORWARD);
                    drive.shooterTwo.setDirection(DcMotorSimple.Direction.FORWARD);

                    drive.shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    drive.shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    PIDFCoefficients pidfCoeffs = new PIDFCoefficients(kP, 0, kD, kF);

                    drive.shooterOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);
                    drive.shooterTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffs);


                    drive.intakeBlocker.setPosition(0.025);

//                    // telemetry
//                    shooter.currentRPMPwr = drive.shooterOne.getVelocity() / 28 * 60;
//                    shooter.wheelRPMPwr = shooter.currentRPM * 1.37;
//                    telemetry.addData("x", poseEstimate.getX());
//                    telemetry.addData("y", poseEstimate.getY());
//                    telemetry.addData("heading", poseEstimate.getHeading());
//                    telemetry.update();
//
//                    drive.intake.setPower(0);
//                    drive.spindexer.setPower(0);
//
//                    shooter.targetVeloPwr = -600;
//
//                    shooter.veloController.setTargetVelocity(shooter.targetVeloPwr);
//                    shooter.veloController.setTargetAcceleration((shooter.targetVeloPwr-shooter.lastTargetVeloPwr) / veloTimerPwr.seconds());
//
//                    shooter.lastTargetVeloPwr = shooter.targetVeloPwr;
                    drive.shooterOne.setVelocity(1340);
                    drive.shooterTwo.setPower(drive.shooterOne.getPower());

                    while (drive.shooterOne.getVelocity() < 840 && opModeIsActive()) {
                        telemetry.addData("timer:", shooterTimer.seconds());
                        telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                        telemetry.update();
                        if (shooterTimer.seconds() > 5) {
                            break;
                        }
                        if (gamepad1.x) {
                            currentMode = Mode.NORMAL_CONTROL;
                        }
                    }

                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(-1100);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.spindexer.setPower(-1);
                    while (drive.spindexer.getCurrentPosition() > -1100 && opModeIsActive()) {
                        telemetry.addData("Velo:", -drive.shooterOne.getVelocity());

                        telemetry.addData("upperBound", TuningController.rpmToTicksPerSecond(TuningController.TESTING_MAX_SPEED * 1.15));
                        telemetry.addData("lowerBound", 0);
                        telemetry.update();
                        telemetry.addData("transfer", drive.spindexer.getCurrentPosition());
                        telemetry.update();
                        if (gamepad1.x) {
                            currentMode = Mode.NORMAL_CONTROL;
                        }
                    }

                    drive.spindexer.setPower(0);
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    counter = 0;
                    currentMode = Mode.NORMAL_CONTROL;
                    break;
                case POWER_SHOTS:
                    if (gamepad1.x) {
                        currentMode = Mode.NORMAL_CONTROL;
                    }

                    drive.shooterOne.setDirection(DcMotorSimple.Direction.FORWARD);
                    drive.shooterTwo.setDirection(DcMotorSimple.Direction.FORWARD);

                    drive.shooterOne.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    drive.shooterTwo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    PIDFCoefficients pidfCoeffsPwr = new PIDFCoefficients(kP, 0, kD, kF);

                    drive.shooterOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffsPwr);
                    drive.shooterTwo.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoeffsPwr);


                    drive.intakeBlocker.setPosition(0.025);

//                    // telemetry
//                    shooter.currentRPMPwr = drive.shooterOne.getVelocity() / 28 * 60;
//                    shooter.wheelRPMPwr = shooter.currentRPM * 1.37;
//                    telemetry.addData("x", poseEstimate.getX());
//                    telemetry.addData("y", poseEstimate.getY());
//                    telemetry.addData("heading", poseEstimate.getHeading());
//                    telemetry.update();
//
//                    drive.intake.setPower(0);
//                    drive.spindexer.setPower(0);
//
//                    shooter.targetVeloPwr = -600;
//
//                    shooter.veloController.setTargetVelocity(shooter.targetVeloPwr);
//                    shooter.veloController.setTargetAcceleration((shooter.targetVeloPwr-shooter.lastTargetVeloPwr) / veloTimerPwr.seconds());
//
//                    shooter.lastTargetVeloPwr = shooter.targetVeloPwr;
                    drive.shooterOne.setVelocity(850);
                    drive.shooterTwo.setPower(drive.shooterOne.getPower());
//
//                    double motorPosPwr = drive.shooterOne.getCurrentPosition();
//                    double motorVeloPwr = drive.shooterOne.getVelocity();
//
//                    double powerPwr = shooter.veloController.update(motorPosPwr, motorVeloPwr);
//
//                    shooter.setPower(powerPwr, drive);

                    while (drive.shooterOne.getVelocity() > -850 && opModeIsActive()) {
                        drive.shooterOne.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(kP, 0, kD, kF));
                        telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                        telemetry.update();
                        if (gamepad1.x) {
                            currentMode = Mode.NORMAL_CONTROL;
                        }
                    }

                    spindexer.ringPositionThree(drive);
                    telemetry.clear();
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                    drive.turn(Math.toRadians(-6));
                    while (drive.shooterOne.getVelocity() > -850 && opModeIsActive()) {
                        telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                        telemetry.update();
                        if (gamepad1.x) {
                            currentMode = Mode.NORMAL_CONTROL;
                        }
                    }
                    telemetry.clear();
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                    spindexer.ringPositionFour(drive);
                    telemetry.clear();
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                    drive.turn(Math.toRadians(-8));
                    while (drive.shooterOne.getVelocity() > -850 && opModeIsActive()) {
                        telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                        telemetry.update();
                        if (gamepad1.x) {
                            currentMode = Mode.NORMAL_CONTROL;
                        }
                    }
                    telemetry.clear();
                    telemetry.addData("Velo:", drive.shooterOne.getVelocity());
                    telemetry.update();
                    spindexer.ringPositionThree(drive);

                    currentMode = Mode.NORMAL_CONTROL;
                    break;

            }



            telemetry.addData("mode", currentMode);

            /*---------------
            Gamepad 1 (devesh)
            ---------------*/

            //drivetrain
            if (gamepad1.y) {
                spindex = !spindex;
                while (gamepad1.y && opModeIsActive());
            }

            if (spindex) {

                if (counter == 0) {
                    telemetry.clear();
                    telemetry.addLine("ayan");
                    telemetry.update();
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(-132);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.spindexer.setPower(-1);
                    while (drive.spindexer.getCurrentPosition() > -132 && opModeIsActive()) {
                        if (gamepad1.left_bumper) {
                            intake = !intake;
                            while (gamepad1.left_bumper && opModeIsActive());
                        }

                        if (intake) {
                            drive.intake.setPower(1);
                            drive.intakeBlocker.setPosition(0.3);
                        } else {
                            drive.intakeBlocker.setPosition(0.025);
                            drive.intake.setPower(0);
                        }
                        driveDirection = new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        );
                        drive.setWeightedDrivePower(driveDirection);
                        drive.getLocalizer().update();
                    }

                    drive.spindexer.setPower(0);
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    counter++;
                    spindex = false;
                } else if (counter == 1) {
                    telemetry.clear();
                    telemetry.addLine("ayan");
                    telemetry.update();
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(-137);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.spindexer.setPower(-1);
                    while (drive.spindexer.getCurrentPosition() > -137 && opModeIsActive()) {
                        if (gamepad1.left_bumper) {
                            intake = !intake;
                            while (gamepad1.left_bumper && opModeIsActive());
                        }

                        if (intake) {
                            drive.intake.setPower(1);
                            drive.intakeBlocker.setPosition(0.3);
                        } else {
                            drive.intakeBlocker.setPosition(0.025);
                            drive.intake.setPower(0);
                        }
                        driveDirection = new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        );
                        drive.setWeightedDrivePower(driveDirection);
                        drive.getLocalizer().update();
                    }

                    drive.spindexer.setPower(0);
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    counter++;
                    spindex = false;
                } else {
                    telemetry.clear();
                    telemetry.addLine("vasisht");
                    telemetry.update();
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    drive.spindexer.setTargetPosition(-60);
                    drive.spindexer.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.spindexer.setPower(-1);
                    while (drive.spindexer.getCurrentPosition() > -40 && opModeIsActive()) {
                        if (gamepad1.left_bumper) {
                            intake = !intake;
                            while (gamepad1.left_bumper && opModeIsActive());
                        }

                        if (intake) {
                            drive.intake.setPower(1);
                            drive.intakeBlocker.setPosition(0.3);
                        } else {
                            drive.intakeBlocker.setPosition(0.025);
                            drive.intake.setPower(0);
                        }
                        driveDirection = new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        );
                        drive.setWeightedDrivePower(driveDirection);
                        drive.getLocalizer().update();
                    }

                    drive.spindexer.setPower(0);
                    drive.spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//                    telemetry.clear();
                    telemetry.addLine("vasisht2");
                    telemetry.update();
                    counter = 0;
//                    telemetry.clear();
                    telemetry.addLine("vasisht3");
                    telemetry.update();
                    spindex = false;

                    telemetry.addLine("vasisht4");
                    telemetry.update();
                }
                telemetry.clear();
                telemetry.addLine("vasisht5");
                telemetry.update();
                spindex = false;


            }


            telemetry.addData("spin:", spindex);


            // wobble goal arm
            if (gamepad1.a) {
                arm = !arm;
                while (gamepad1.a && opModeIsActive());
            }



            if (!arm) {
                drive.wobbleGoalArmOne.setPosition(.2);
                drive.wobbleGoalArmTwo.setPosition(.2);
            } else {
                drive.wobbleGoalArmOne.setPosition(.8);
                drive.wobbleGoalArmTwo.setPosition(.8);
            }


            // wobble goal grip
            if (gamepad1.b) {
                grip = !grip;
                while (gamepad1.b && opModeIsActive());
            }

            if (grip) {
                drive.wobbleGoalGripper.setPosition(.7);
            } else {
                drive.wobbleGoalGripper.setPosition(.3);
            }

            /*---------------
            Gamepad 2 (ayan)
            ---------------*/

            // shooter




            // transfer


            //hdhsuiwd daksjdfnadN ; 4SD IF LOOOPA  dhhdaopfjejkkkfs gfs;::: () { }

            // intake
            //drive.intake.setPower(gamepad1.left_trigger);
            if (gamepad1.left_bumper) {
                intake = !intake;
                while (gamepad1.left_bumper && opModeIsActive());
            }

            if (intake) {
                drive.intake.setPower(1);
                drive.intakeBlocker.setPosition(0.3);
            } else {
                drive.intakeBlocker.setPosition(0.025);
                drive.intake.setPower(0);
            }

            //reverse transferve
//            if (gamepad1.left_bumper) {
//                drive.spindexer.setPower(1);
//            }
            if (gamepad1.right_trigger != 0) {
                drive.ringGate.setPosition(0);
            } else {
                drive.ringGate.setPosition(.35);
            }




            drive.setWeightedDrivePower(driveDirection);
            drive.getLocalizer().update();




            //escalar las montanas
        }
    }
}
