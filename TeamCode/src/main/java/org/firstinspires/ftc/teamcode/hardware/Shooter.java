package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.hardware.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.util.VelocityPIDFController;

public class Shooter {

    public static PIDCoefficients MOTOR_VELO_PID = new PIDCoefficients(0.004, 0, 0.0001);

    public static double kV = 0.00053;
    public static double kA = 0.0007;
    public static double kStatic = 0;

    public double currentRPM;
    public double wheelRPM;

    public final ElapsedTime veloTimer = new ElapsedTime();
    public double targetVelo = 0;
    public double lastTargetVelo = 0.0;

    public double currentRPMPwr;
    public double wheelRPMPwr;

    public final ElapsedTime veloTimerPwr = new ElapsedTime();
    public double targetVeloPwr = 0;
    public double lastTargetVeloPwr = 0.0;


    public final VelocityPIDFController veloController = new VelocityPIDFController(MOTOR_VELO_PID, kV, kA, kStatic);

    public void setPower(double input, SampleMecanumDrive drive) {
        drive.shooterOne.setPower(input);
        drive.shooterTwo.setPower(input);
    }

    public void setMode(DcMotor.RunMode mode, SampleMecanumDrive drive) {
        for (DcMotorEx motor : drive.shooterMotors) {
            motor.setMode(mode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior, SampleMecanumDrive drive) {
        for (DcMotorEx motor : drive.shooterMotors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

}
