package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoRUn", group="Linear OpMode")
public class AutoRun extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor viper = null;
    private Servo extendo = null;
    private Servo elbow = null;
    private Servo roller = null;
    private Servo claw = null;
    boolean rolling = false;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "rightback"); //rightback
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "rightfront"); //rightfront`
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "leftfront"); //leftfront
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "leftback"); //leftback

        redLED = hardwareMap.get(DigitalChannel.class, "redled");//7
        greenLED = hardwareMap.get(DigitalChannel.class, "greenled");//6

        //Initialize the Servo variables
        extendo = hardwareMap.get(Servo.class, "extendo"); // chub 0
        roller = hardwareMap.get(Servo.class, "roller"); // chub 1
        elbow = hardwareMap.get(Servo.class, "elbow"); // chub 5
        claw = hardwareMap.get(Servo.class, "claw"); // ehub 3

        viper = hardwareMap.get(DcMotor.class, "viper");
        viper.setDirection(DcMotor.Direction.REVERSE);
        viper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        double extendoPosition = XBot.EXTENDO_MIN; // Midpoint for extendo
        double elbowPosition = XBot.ELBOW_VERTICAL;   // Midpoint for elbow
        double rollerPosition = XBot.ROLLER_STOP;  // Midpoint for roller
        double clawPosition = XBot.CLAW_OPEN;  // position for claw opening

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Extendo Position", extendoPosition);
        telemetry.addData("Elbow position", elbowPosition);
        telemetry.addData("Viper Position",  "%7d", viper.getCurrentPosition());
        telemetry.update();
        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        Pose2d beginPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(12.5, 63, Math.toRadians(-90)));

        waitForStart();
        runtime.reset();

        Action blueRightAction = drive.actionBuilder(new Pose2d(12.5, 63, Math.toRadians(-90)))
                .lineToYConstantHeading(31) // drives to the chamber
                // drops preloaded specimen on the chamber
                .afterDisp(10, () -> {
                    // Move viperSlide to a specific height
                    dropSpecimen();
                })
                //go to pick next yellow sample
                .splineToLinearHeading(new Pose2d(12.5, 37, Math.toRadians(165)), Math.toRadians(-90)) // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                .strafeTo(new Vector2d(26,32)) // moves in the direction of the sample and extendo extends
//                // extendo takes in the sample
//                .waitSeconds(1)
//                //drop the sample
//                .splineTo(new Vector2d(55, 56), Math.toRadians(45))
//                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
//                //Pick next one
//                .splineToLinearHeading(new Pose2d(38, 38, Math.toRadians(135)), Math.toRadians(-90)) // robot aligns itself to get the second sample
//                .waitSeconds(2) // extendo extends and takes in the second sample
//                //drop again
//                .splineTo(new Vector2d(55, 56), Math.toRadians(45)) // splines goes drop first sample in the high basket!
//                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
//                //Pick again
//                .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(135)), Math.toRadians(-90)) // robot aligns itself to get the second sample
//                .waitSeconds(2) // extendo extends and takes in the second sample
//                //drop again
//                .splineTo(new Vector2d(55, 56), Math.toRadians(45)) // splines goes drop first sample in the high basket!
//                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
//                .strafeTo(new Vector2d(50, 38)) // splines goes drop first sample in the high basket!
//                .splineToLinearHeading(new Pose2d(21.5, 10, Math.toRadians(0)), Math.toRadians(90)) // splines to rung for level 1 ascent (3 points)
                .build();

            Actions.runBlocking(blueRightAction);

            // Telemetry
            telemetry.addData("AutoRun",  "DONE");

            // Show the elapsed game time and wheel power.
            telemetry.update();
    }

    private void dropSpecimen() {
        //(viper slide is at drop sepcimen level)
        if (opModeIsActive()) {
            //drop specimen level viper slide
            viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.DROPPED_SPECIMEN, 1000);
            sleep(200);
            claw.setPosition(XBot.CLAW_OPEN);
            sleep(500);
            viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_HOME, 1000);
        }
        //move the viper slide to DROPPED SPECIMEn
        //Open claw
        //return viper home
    }

    private void viperDrive(double maxSpeed, int inches, double timeoutS) {
        int newTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            newTarget = viper.getCurrentPosition() + (int)(inches * XBot.COUNTS_PER_INCH);
            viper.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            viper.setPower(Math.abs(maxSpeed));

            // keep looping while we are still active, and there is time left, and viper motor is running.
            // Note: We use (isBusy()) in the loop test, which means that when viper motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (viper.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Viper Running to",  " %7d", newTarget);
                telemetry.addData("Currently at",  " at %7d", viper.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            viper.setPower(0);
        }
    }


    /* PID CONTROLLER CODE

    private void viperDrive(double maxSpeed, int inches, double timeoutS) {
        int targetPosition;
        double Kp = 0.1; // Proportional gain (tune this value)
        double Ki = 0.01; // Integral gain (tune this value)
        double Kd = 0.01; // Derivative gain (tune this value)
        double previousError = 0; // Store the error from the previous iteration
        double integralSum = 0; // Accumulate the error over time
        double power;
        double error;
        double derivative;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Calculate the target position
            targetPosition = viper.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            viper.setTargetPosition(targetPosition);

            // Turn On RUN_TO_POSITION
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();

            // Loop until timeout or position reached
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (viper.isBusy())) {
                // Calculate the error
                error = targetPosition - viper.getCurrentPosition();

                // Accumulate the error for the integral term
                integralSum += error;

                // Calculate the derivative (rate of change of error)
                derivative = error - previousError;

                // Compute control power
                power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

                // Limit power to the maximum speed
                power = Math.max(-Math.abs(maxSpeed), Math.min(Math.abs(maxSpeed), power));

                // Set motor power
                viper.setPower(power);

                // Update the previous error for the next loop
                previousError = error;

                // Display information for the driver
                telemetry.addData("Viper Target", targetPosition);
                telemetry.addData("Current Position", viper.getCurrentPosition());
                telemetry.addData("Error", error);
                telemetry.addData("Integral Sum", integralSum);
                telemetry.addData("Derivative", derivative);
                telemetry.addData("Power", power);
                telemetry.update();
            }

            // Stop all motion
            viper.setPower(0);
        }
    }

*/

    private void viperDriveToPositionInInches(double maxSpeed, double inches, double timeoutS) {
        int newTarget;
        double Kp = 0; // Proportional gain (tune this value)
        double Ki = 0; // Integral gain (tune this value)
        double Kd = 0; // Derivative gain (tune this value)
        double previousError = 0; // Store the error from the previous iteration
        double integralSum = 0; // Accumulate the error over time
        double power;
        double error;
        double derivative;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
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
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (viper.isBusy())) {
                // Calculate the error
                error = newTarget - viper.getCurrentPosition();

                // Accumulate the error for the integral term
                integralSum += error;

                // Calculate the derivative (rate of change of error)
                derivative = error - previousError;

                // Compute control power
                power = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + maxSpeed; //Remove maxSpeed after tuning

                // Limit power to the maximum speed
                power = Math.max(-Math.abs(maxSpeed), Math.min(Math.abs(maxSpeed), power));

                // Set motor power
                viper.setPower(power);

                // Update the previous error for the next loop
                previousError = error;

                // Display information for the driver
                telemetry.addData("Viper Target", newTarget);
                telemetry.addData("Current Position", viper.getCurrentPosition());
                telemetry.addData("Error", error);
                telemetry.addData("Integral Sum", integralSum);
                telemetry.addData("Derivative", derivative);
                telemetry.addData("Power", power);
                telemetry.update();
            }

            // Stop all motion;
            viper.setPower(0.015);
        }
    }

    private void viperDriveRunToPosition(double speed, int position, double timeoutS) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            viper.setTargetPosition(position);

            // Turn On RUN_TO_POSITION
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            viper.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and viper motor is running.
            // Note: We use (isBusy()) in the loop test, which means that when viper motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            while (opModeIsActive()  && (runtime.seconds() < timeoutS) && (viper.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Viper Running to",  " %7d", position);
                telemetry.addData("Currently at",  " at %7d", viper.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            viper.setPower(0);
        }
    }
}

