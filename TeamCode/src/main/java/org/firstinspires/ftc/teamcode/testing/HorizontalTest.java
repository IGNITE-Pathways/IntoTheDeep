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
public class HorizontalTest extends OpMode {
    private PIDFController controller;
    public static double p = 0.05, i = 0, d = 0.0005;
    public static double f = 0.00004;

    public static double targetPositionInInches = 0.0;
    public static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!

    private DcMotorEx intakeDCMotor;

    @Override
    public void init() {
        controller = new PIDFController(p, i, d, f);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        intakeDCMotor = hardwareMap.get(DcMotorEx.class, "intakedc");
        intakeDCMotor.setDirection(DcMotor.Direction.REVERSE);
        controller.setTolerance(5); // optional: how close to setpoint you want to be in ticks
    }

    @Override
    public void loop() {
        controller.setPIDF(p, i, d, f);
        double slidePos = intakeDCMotor.getCurrentPosition();

        double pid = controller.calculate(slidePos, targetPositionInInches * TICKS_PER_INCH);

        double power = pid;

        intakeDCMotor.setPower(power);

        telemetry.addData("targetPos", targetPositionInInches * TICKS_PER_INCH);
        telemetry.addData("currentPos", slidePos);
        telemetry.update();
    }
}