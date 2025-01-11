package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class VerticalTest extends OpMode {
    private PIDFController controller;
    public static double p = 0.03, i = 0, d = 0.0001;
    public static double f = 0.00004;

    public static double targetPositionInInches = 0.0;

    public static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!

    private DcMotorEx outtakeDCLeft;
    private DcMotorEx outtakeDCRight;

    @Override
    public void init() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outtakeDCLeft = hardwareMap.get(DcMotorEx.class, "outtakedcleft");
        outtakeDCRight = hardwareMap.get(DcMotorEx.class, "outtakedcright");

        outtakeDCRight.setDirection(DcMotor.Direction.FORWARD);
        outtakeDCLeft.setDirection(DcMotor.Direction.REVERSE);

//        outtakeDCRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        outtakeDCRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeDCRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        outtakeDCLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        outtakeDCLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        outtakeDCLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller.setTolerance(5); // optional: how close to setpoint you want to be in ticks
    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        double slidePos = outtakeDCLeft.getCurrentPosition();
        double pid = controller.calculate(slidePos, targetPositionInInches * TICKS_PER_INCH);

        double power = pid + f;

        outtakeDCLeft.setPower(power);
        outtakeDCRight.setPower(power);

        telemetry.addData("targetPos", targetPositionInInches * TICKS_PER_INCH);
        telemetry.addData("currentPos", slidePos);
        telemetry.update();
    }
}