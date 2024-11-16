package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    static final double     COUNTS_PER_MOTOR_REV    = 537.7;// 384.5; // 1425.1;    // eg: Motor Encoder 312, 117, 435
    static final double     PULLEY_DIAMETER_INCHES   = 1.5 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH  = (COUNTS_PER_MOTOR_REV) / (PULLEY_DIAMETER_INCHES * Math.PI); //81.6 ticks per inch
    static final double     VIPER_DRIVE_SPEED             = 1.0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftback");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback");

        //Initialize the Servo variables
//        extendo = hardwareMap.get(Servo.class, "extendo");
//        elbow = hardwareMap.get(Servo.class, "elbow");
//        roller = hardwareMap.get(Servo.class, "roller");

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
        telemetry.addData("Status", "Initialized");
//        telemetry.addData("COUNTS_PER_INCH", "%7d", COUNTS_PER_INCH);
        telemetry.addData("Viper Position",  "%7d", viper.getCurrentPosition());
        telemetry.update();

        double extendoPosition = 0.5;
        double elbowPosition = 0.5;
        double rollerPosition = 0.5;
        double JOYSTICK_SENSITIVITY = 0.01;

        int viperPosition = 0;

        waitForStart();
        runtime.reset();

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


            // Control the extendo servo with the left stick Y-axis
            extendoPosition += -gamepad1.left_stick_y * JOYSTICK_SENSITIVITY;

            // Control the wrist servo with the right stick Y-axis
            elbowPosition += -gamepad1.right_stick_y * JOYSTICK_SENSITIVITY;

            // Control the roller servo with the left stick X-axis
            rollerPosition += gamepad1.left_stick_x * JOYSTICK_SENSITIVITY;

            // Control the viper position with the right stick Y-axis
            viperPosition += -(int) gamepad2.right_stick_y * 100;

            if (gamepad2.x) {
                viperDriveRunToPosition(VIPER_DRIVE_SPEED, 0, 100);
//                viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (gamepad2.y) {
                //Move Viper Slide
                viperDriveToPositionInInches(VIPER_DRIVE_SPEED,  26, 100.0);  // S1: Forward 47 Inches with 5 Sec timeout
            }
            // Clamp positions to stay within servo range (0 to 1)
            extendoPosition = Math.min(Math.max(extendoPosition, 0), 1);
            elbowPosition = Math.min(Math.max(elbowPosition, 0), 1);
            rollerPosition = Math.min(Math.max(rollerPosition, 0), 1);

            // Update servo positions
//            extendo.setPosition(extendoPosition);
//            elbow.setPosition(elbowPosition);
//            roller.setPosition(rollerPosition);
            if (!isViperPositionClose(viperPosition)) {
                viperDrive(VIPER_DRIVE_SPEED, (int)(viperPosition / COUNTS_PER_INCH), 100);
            }

            // Telemetry
//            telemetry.addData("Extendo Position", extendoPosition);
//            telemetry.addData("Wrist Position", elbowPosition);
//            telemetry.addData("Roller Position", rollerPosition);
            telemetry.addData("GAME_PAD_RIGHT_Y", "%7d", viperPosition);
            telemetry.addData("Viper Position",  "%7d", viper.getCurrentPosition());

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

    private void viperDrive(double speed, int inches, double timeoutS) {
        int newTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            newTarget = viper.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);
            viper.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            viper.setPower(Math.abs(speed));

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

    private void viperDriveToPositionInInches(double speed, int inches, double timeoutS) {
        int newTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            newTarget = (int)(inches * COUNTS_PER_INCH);
            viper.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            viper.setPower(Math.abs(speed));

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
            while (opModeIsActive()  && (runtime.seconds() < timeoutS)&& (viper.isBusy())) {
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

