package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Viper {
    private DcMotor viper = null;
    private ElapsedTime runtime = new ElapsedTime();

    public Viper(HardwareMap hardwareMap) {
        viper = hardwareMap.get(DcMotor.class, "viper");
        viper.setDirection(DcMotor.Direction.REVERSE);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public class GetReadyToDropSpecimen implements Action {
        private boolean initialized = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
//                motor.setPower(0.8);
                viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_DROP_SPECIMEN, 1000);
                initialized = true;
            }

            double pos = viper.getCurrentPosition();
            packet.put("viper position", pos);
            return pos > XBot.VIPER_DROP_SPECIMEN - 5;
        }

        private void viperDriveToPositionInInches(double maxSpeed, double inches, double timeoutS) {
            int newTarget;
            double Kp = 0; // Proportional gain (tune this value)
            double Ki = 0; // Integral gain (tune this value)
            double Kd = 0; // Derivative gain (tune this value)
            double power;

            // Ensure that the OpMode is still active
                newTarget = (int)(inches * XBot.COUNTS_PER_INCH);
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
                    
                    // Display information for the driver
//                    telemetry.addData("Viper Target", newTarget);
//                    telemetry.addData("Current Position", viper.getCurrentPosition());
//                    telemetry.addData("Power", power);
//                    telemetry.update();

                // Stop all motion;
                viper.setPower(0.015);
            }
        }
    }

    public Action getReadyToDropSpecimen() {
        return new GetReadyToDropSpecimen();
    }

//    public Action dropSpecimen() {
//        return new TodoAction();
//    }

}

//private void dropSpecimen() {
//    //(viper slide is at drop sepcimen level)
//    if (opModeIsActive()) {
//        //drop specimen level viper slide
//        viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.DROPPED_SPECIMEN, 1000);
//        sleep(200);
//        claw.setPosition(XBot.CLAW_OPEN);
//        sleep(500);
//        viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_HOME, 1000);
//    }
//    //move the viper slide to DROPPED SPECIMEn
//    //Open claw
//    //return viper home
//}
