package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.XBot;

@TeleOp
@Config
@Disabled
public class HorizontalTestNoPID extends OpMode {

//    public static double targetPosition = 0;
    public static double targetPositionInInches = 0.0;

    public static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotorEx intakeDCMotor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeDCMotor = hardwareMap.get(DcMotorEx.class, "intakedc");
        intakeDCMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeDCMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDCMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if (Math.abs(intakeDCMotor.getCurrentPosition() - targetPositionInInches * TICKS_PER_INCH) > 10) {
            driveToPosition(0.5, targetPositionInInches, 1000);
        }
        telemetry.addData("targetPos", targetPositionInInches * TICKS_PER_INCH);
        telemetry.addData("currentPos", intakeDCMotor.getCurrentPosition());
        telemetry.update();
    }

    private void driveToPosition(double maxSpeed, double inches, double timeoutMilliSeconds) {
        // Ensure that the OpMode is still active
        int newTarget = (int)(inches * TICKS_PER_INCH);
        intakeDCMotor.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        intakeDCMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        intakeDCMotor.setPower(Math.abs(maxSpeed));

        // keep looping while we are still active, and there is time left, and intakeDC motor is running.
        // Note: We use (isBusy()) in the loop test, which means that when intakeDC motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        while ((runtime.milliseconds() < timeoutMilliSeconds) && (intakeDCMotor.isBusy())) {
            // Set motor power
            intakeDCMotor.setPower(maxSpeed);
        }
        // Stop all motion;
        intakeDCMotor.setPower(0);
    }

    public int getPosition() {
        return intakeDCMotor.getCurrentPosition();
    }
}