package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class WobbleGoalMech extends SampleMecanumDrive{

    double power = 0;

    public WobbleGoalMech(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void setPosition(double input) {
//        super.wobbleGoalArm.setPosition(input);
    }

    public void grip(double state) {
        if (state == 0) {
            super.wobbleGoalGripper.setPosition(.1);
        } else {
            super.wobbleGoalGripper.setPosition(.9);
        }
    }


}
