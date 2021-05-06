package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous(name = "FebOpenCVTestingGround")
public class FebOpenCVTestingGround extends LinearOpMode {

    double ringLevel;

    @Override
    public void runOpMode() {
        RingStackDetector detector = new RingStackDetector(hardwareMap, telemetry, .12, .39, .46);
        detector.init();
        waitForStart();
    }

}
