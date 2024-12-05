package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.opencv.core.Mat;

public class Viper {
    private DcMotor viper = null;
    private Servo claw = null;
    private ElapsedTime runtime = new ElapsedTime();

    public Viper(HardwareMap hardwareMap) {
        viper = hardwareMap.get(DcMotor.class, "viper");
        viper.setDirection(DcMotor.Direction.REVERSE);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        claw = hardwareMap.get(Servo.class, "claw"); // ehub 3
    }

    public void initialize() {
        claw.setPosition(XBot.CLAW_CLOSE);
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
                double pos = viper.getCurrentPosition();
                packet.put("viper position", pos);
                return Math.abs(pos - inches) < 0.5;
            }
        };
    }

    public Action getReadyToDropSpecimen() {
        return new Action () {
                private boolean initialized = false;

                @Override
                public boolean run(@NonNull TelemetryPacket packet) {
                    if (!initialized) {
                        driveToPosition(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_DROP_SPECIMEN, 1000, packet);
                        initialized = true;
                    }

                    double pos = viper.getCurrentPosition();
                    packet.put("viper position", pos);
                    return Math.abs(pos - XBot.VIPER_DROP_SPECIMEN) < 0.5;
                }
            };
    }

    public Action testAction() {
        return new Action() {
            private int count = 0;

            @Override
            public boolean run(TelemetryPacket packet) {
                count++;
                System.out.println("Run call #" + count);
                packet.put("Run call #: ", count);
                boolean isComplete = (count > 10); // Mark action as complete after 10 iterations
                System.out.println("Returning: " + isComplete);
                packet.put("Returning: ", isComplete);
                System.out.flush();
                return isComplete;
            }
        };
    }

//    public Action dropSpecimen() {
//        return new Action () {
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                driveToPosition(XBot.VIPER_DRIVE_SPEED, XBot.DROPPED_SPECIMEN, 1000, packet);
//                try {
//                    sleep(200);
//                    claw.setPosition(XBot.CLAW_OPEN);
//                    sleep(200);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//                driveToPosition(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_HOME, 1000, packet);
//                return false;
//            }
//        };
//    }

    public Action openClaw() {
        return new Action () {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(XBot.CLAW_OPEN);
                return false;
                //return Math.abs(claw.getPosition() - XBot.CLAW_OPEN) > 0.1;
            }
        };
    }

    public Action closeClaw() {
        return new Action () {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                claw.setPosition(XBot.CLAW_CLOSE);
//                return false;
                return Math.abs(claw.getPosition() - XBot.CLAW_CLOSE) > 0.1;
            }
        };
    }

    private void driveToPosition(double maxSpeed, double inches, double timeoutS, TelemetryPacket packet) {
        // Ensure that the OpMode is still active
        int newTarget = (int)(inches * XBot.COUNTS_PER_INCH);
        viper.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        viper.setPower(Math.abs(maxSpeed));

        // keep looping while we are still active, and there is time left, and viper motor is running.
        // Note: We use (isBusy()) in the loop test, which means that when viper motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        while ((runtime.seconds() < timeoutS) && (viper.isBusy())) {
            // Set motor power
            viper.setPower(maxSpeed);
            packet.put("viper position", viper.getCurrentPosition());
        }
        // Stop all motion;
        viper.setPower(0.015);
    }
}
