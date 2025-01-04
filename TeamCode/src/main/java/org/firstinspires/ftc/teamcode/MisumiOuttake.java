package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MisumiOuttake {

    private DcMotor outtakeLeft = null;
    private DcMotor outtakeRight = null;
    private Servo gearLeft = null;
    private Servo gearRight = null;
    private Servo Claw = null;
    private ElapsedTime runtime = new ElapsedTime();

    public MisumiOuttake(HardwareMap hardwareMap) {
        outtakeLeft = hardwareMap.get(DcMotor.class, "viper");
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeRight = hardwareMap.get(DcMotor.class, "viper");
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gearLeft = hardwareMap.get(Servo.class, "claw");

        gearRight = hardwareMap.get(Servo.class, "claw");

        Claw = hardwareMap.get(Servo.class, "claw");

    }

    public void initialize() {
        Claw.setPosition(XBot.CLAW_CLOSE);
    }

    public Action driveToPositionInInches(double inches) {
        return new Action () {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    driveToPosition(XBot.VIPER_DRIVE_SPEED, inches, 1000, packet);
                    initialized = true;
                }
                double pos = outtakeLeft.getCurrentPosition();
                packet.put("outtake left motor position", pos);
                double pos2 = outtakeRight.getCurrentPosition();
                packet.put("outtake right motor position", pos2);
                return Math.abs(pos - inches) < 0.5;
            }
        };
    }

    private void driveToPosition(double maxSpeed, double inches, double timeoutS, TelemetryPacket packet) {
        // Ensure that the OpMode is still active
        int newTarget = (int)(inches * XBot.COUNTS_PER_INCH);
        outtakeLeft.setTargetPosition(newTarget);
        outtakeRight.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        outtakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();

        outtakeLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // keep looping while we are still active, and there is time left, and viper motor is running.
        // Note: We use (isBusy()) in the loop test, which means that when viper motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        while ((runtime.seconds() < timeoutS) && (outtakeLeft.isBusy()) && (outtakeRight.isBusy())) {
            // Set motor power
            outtakeLeft.setPower(maxSpeed);
            outtakeRight.setPower(maxSpeed);
            double pos = outtakeLeft.getCurrentPosition();
            packet.put("outtake left motor position", pos);
            double pos2 = outtakeRight.getCurrentPosition();
            packet.put("outtake right motor position", pos2);        }
        // Stop all motion;
        outtakeLeft.setPower(0.015);
        outtakeRight.setPower(0.015);
    }

}

