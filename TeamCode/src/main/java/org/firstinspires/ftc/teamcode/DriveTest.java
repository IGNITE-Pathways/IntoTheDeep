package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="DriveTest", group="Linear OpMode")
public class DriveTest extends LinearOpMode {

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


    private enum State {
        SAMPLE,
        SPECIMEN
    }
    private DriveTest.State state = State.SAMPLE; // Default no state


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftback");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback");
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

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)


        double extendoPosition = XBot.EXTENDO_MIN; // Midpoint for extendo
        double elbowPosition = XBot.ELBOW_VERTICAL;   // Midpoint for elbow
        double rollerPosition = XBot.ROLLER_STOP;  // Midpoint for roller
        double clawPosition = XBot.CLAW_OPEN;  // position for claw opening

//        int viperPosition = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Extendo Position", extendoPosition);
        telemetry.addData("Elbow position", elbowPosition);

//        telemetry.addData("COUNTS_PER_INCH", "%7d", COUNTS_PER_INCH);
        telemetry.addData("Viper Position",  "%7d", viper.getCurrentPosition());
        telemetry.update();
        waitForStart();
        runtime.reset();
        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower = axial - lateral + yaw;
            double rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

// intaking sample

            if (gamepad2.b) {           //circle on sony controller
                state = State.SAMPLE;
                extendoPosition = XBot.EXTENDO_MAX;
                elbowPosition = XBot.ELBOW_MAX;
            }
            if (gamepad2.a) {           // x on sony controller
                state = State.SAMPLE;
                extendoPosition = XBot.EXTENDO_MIN;
                elbowPosition = XBot.ELBOW_MIN;
                extendo.setPosition(extendoPosition);
                elbow.setPosition(elbowPosition);
                sleep(1000);
//                // roll the roller to drop the sample
//                rollerPosition = XBot.ROLLER_BACKWARD;
//                roller.setPosition(rollerPosition);
//                sleep(1000);
                // make elbow vertical once the sample is dropped
                elbowPosition = XBot.ELBOW_VERTICAL; // bringing the elbow a little bit up after it drops the sample
                elbow.setPosition(elbowPosition);
            }


//            if (gamepad2.dpad_up) {
//                extendoPosition += XBot.SERVO_INCREMENT; // Increase position
//            } else if (gamepad2.dpad_down) {
//                extendoPosition -= XBot.SERVO_INCREMENT; // Decrease position
//            }
//
//            // Control for Elbow Servo
//            if (gamepad2.dpad_right) {
//                elbowPosition += XBot.SERVO_INCREMENT; // Increase position
//            } else if (gamepad2.dpad_left) {
//                elbowPosition -= XBot.SERVO_INCREMENT; // Decrease position
//            }

            if ((gamepad2.right_trigger < 0.5) && (gamepad2.left_trigger < 0.5)) { // both of them aren't touched
                rollerPosition = XBot.ROLLER_STOP;
            } else if (gamepad2.right_trigger > 0.5) {
                rollerPosition = XBot.ROLLER_BACKWARD;
            } else if (gamepad2.left_trigger > 0.5) {
                rollerPosition = XBot.ROLLER_FORWARD;
            }

            //

            if (gamepad2.left_bumper) {
                claw.setPosition(XBot.CLAW_CLOSE);
            } else if (gamepad2.right_bumper) {
                claw.setPosition(XBot.CLAW_OPEN);
            }


            extendoPosition = Math.min(Math.max(extendoPosition, XBot.EXTENDO_MAX), XBot.EXTENDO_MIN);
            elbowPosition = Math.min(Math.max(elbowPosition, XBot.ELBOW_MAX), XBot.EXTENDO_MIN);
            //rollerPosition = Math.min(Math.max(rollerPosition, XBot.SERVO_MIN), XBot.SERVO_MAX);

            // Update servo positions
            extendo.setPosition(extendoPosition);
            elbow.setPosition(elbowPosition);
            roller.setPosition(rollerPosition);

            // Control the viper position with the right stick Y-axis
//            viperPosition += -(int) gamepad2.right_stick_y * 100;


            if (gamepad2.dpad_up) {//high basket dpad up
                state = State.SAMPLE;
                viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_DROP_SAMPLE_HIGHER_BUCKET - 2.0, 1000);
                clawPosition = XBot.CLAW_FULLY_OPEN;
                claw.setPosition(clawPosition);
            } else if (gamepad2.dpad_down) {//high chamber dpad down
                if (state == State.SPECIMEN) {
                    viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_DROP_SPECIMEN, 1000);
                } else if (state == State.SAMPLE) {
                    elbowPosition = XBot.ELBOW_VERTICAL - 0.5;
                }
            } else if (gamepad2.x) { //square on sony controller,   pick specimen off wall
                state = State.SPECIMEN;
                viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_PICK_SPECIMEN, 1000);
            } else if (gamepad2.y) { //triangle on sony controller
                if (state == State.SPECIMEN) {
                    dropSpecimen();
                } else if (state == State.SAMPLE) {
                    viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_DROP_SAMPLE_HIGHER_BUCKET, 1000);
                }
            } else if (gamepad2.dpad_right) { //return viper home
                state = State.SPECIMEN;
                clawPosition = XBot.CLAW_CLOSE;
                claw.setPosition(clawPosition);
                sleep(200);
                viperDriveToPositionInInches(XBot.VIPER_DRIVE_SPEED, XBot.VIPER_HOME, 1000);
            }

            //Set the color based on state
            //SAMPLE = RED, SPECIMEN = GREEN
            switch (state) {
                case SAMPLE:
                    greenLED.setState(false);
                    redLED.setState(true);
                    break;
                case SPECIMEN:
                    greenLED.setState(true);
                    redLED.setState(false);
            }


//            if (!isViperPositionClose(viperPosition)) {
//                viperDrive(VIPER_DRIVE_SPEED, (int)(viperPosition / COUNTS_PER_INCH), 100);
//            }

            // Telemetry
            telemetry.addData("GAME State", state);
            telemetry.addData("Extendo Position", extendoPosition);
            telemetry.addData("Elbow Position", elbowPosition);
            telemetry.addData("Roller Position", rollerPosition);
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("GAME_PAD_RIGHT_Y",  gamepad2.right_stick_y);
            telemetry.addData("Viper Position",  "%7d", viper.getCurrentPosition());
            telemetry.addData("Viper Position Inches", (viper.getCurrentPosition() / XBot.COUNTS_PER_INCH));

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    private boolean isViperPositionClose(double viperPosition) {
       int currentPosition = viper.getCurrentPosition();
       if ((viperPosition < currentPosition + 10) && (viperPosition > currentPosition - 10)) return true;
       if (viperPosition > 3000) return true;
       if (viperPosition < 0) return true;
       return false;
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
                power = (Kp * error) + (Ki * integralSum) + (Kd * derivative);


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

