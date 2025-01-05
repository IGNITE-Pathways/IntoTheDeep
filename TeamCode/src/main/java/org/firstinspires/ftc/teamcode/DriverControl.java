package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Linear OpMode")
public class DriverControl extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //Vertical Misumis / OUTTAKE
    private DcMotor outtakeDCRight = null;
    private DcMotor outtakeDCLeft = null;
    private Servo outtakeServoRight = null;
    private Servo outtakeServoLeft = null;
    private Servo outtakeClaw = null;

    private double robotSpeed = 1.0;

    Diffy diffy = null;
    Extendo extendo = null;

    private enum State {
        SAMPLE,
        SPECIMEN
    }
    private DriverControl.State state = State.SAMPLE; // Default no state

    @Override
    public void runOpMode() {
        diffy = new Diffy(hardwareMap);
        extendo = new Extendo(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront"); //ehub 0
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftback"); //ehub 1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront"); //chub 2
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback"); //chub 3 //Encoder used for ODO Y

        outtakeDCRight = hardwareMap.get(DcMotor.class, "outtakedcright"); //chub 1
        outtakeDCLeft = hardwareMap.get(DcMotor.class, "outtakedcleft"); //ehub 2

        //Initialize the Servo variables
        outtakeServoRight = hardwareMap.get(Servo.class, "outtakeservoright"); // chub 5
        outtakeServoLeft = hardwareMap.get(Servo.class, "outtakeservoleft"); // ehub 0
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeclaw"); // ehub 1

        outtakeDCRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeDCLeft.setDirection(DcMotor.Direction.FORWARD);

        outtakeDCRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDCLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        initializeSystems();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Outtake Motor Pos",  "%7d: %7d", outtakeDCLeft.getCurrentPosition(), outtakeDCRight.getCurrentPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();

        double lastDiffyDegreesChanged = runtime.milliseconds();
        double lastDiffyAngleChanged = runtime.milliseconds();
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

//            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * robotSpeed);
            rightFrontDrive.setPower(rightFrontPower * robotSpeed);
            leftBackDrive.setPower(leftBackPower * robotSpeed);
            rightBackDrive.setPower(rightBackPower * robotSpeed);

            if (gamepad2.circle) { //PICK SAMPLE //B
                //Extends horizontal slides and rotate claw to pickup
                state = State.SAMPLE;
                extendo.extendFully();
//                diffy.moveToPickPosition();
            }
            if (gamepad2.square) { //PICK SPECIMEN //X
                //extend h-misumi out, move diffy down
                state = State.SPECIMEN;
//                diffy.moveToTransferPosition();
            }

            if (gamepad2.x) { //A
                //If intake sample is yellow -- move diffy up, return h-misumi, xfer, raise v-misumi
                //else any other sample -- move diffy up, bring h-misumi back
                diffy.pickSample();
            }

            if ((Math.abs(gamepad2.right_stick_x) >= 0.5) && ((runtime.milliseconds() - lastDiffyDegreesChanged) > 500) ) {
                int sign = (gamepad2.right_stick_x == 0) ? 0 : (gamepad2.right_stick_x > 0) ? 1 : -1;
                diffy.rotate(24.5 * sign);
                lastDiffyDegreesChanged = runtime.milliseconds();
            }

            if ((Math.abs(gamepad2.right_stick_y) >= 0.5) && ((runtime.milliseconds() - lastDiffyAngleChanged) > 200) ) {
                int sign = (gamepad2.right_stick_y == 0) ? 0 : (gamepad2.right_stick_y > 0) ? 1 : -1;
                diffy.rotateUp(45 * sign);
                lastDiffyAngleChanged = runtime.milliseconds();
            }

            // Telemetry
            telemetry.addData("GAME State", state);
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //OUTTAKE
            telemetry.addData("Outtake Motor Pos",  "%7d: %7d", outtakeDCLeft.getCurrentPosition(), outtakeDCRight.getCurrentPosition());

            //INTAKE
            telemetry.addData("diffy Position, Left:", "%4.2f, Right: %4.2f", diffy.diffyLeft.getPosition(), diffy.diffyRight.getPosition());
            telemetry.addData("diffyDegrees", "%7d", diffy.diffyDegrees);
            telemetry.addData("diffyVerticalAngle", "%7d", diffy.diffyVerticalAngle);
            telemetry.addData("intake Claw Position", "%4.2f", diffy.intakeClaw.getPosition());
            telemetry.addData("intake Sensor, Color: " + diffy.getSampleColor(), "Distance: %4.2f", diffy.intakeSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("intake Motor: ", "%7d", extendo.getPosition());

            //MOTORS
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }

    private void initializeSystems() {
        telemetry.addData("Initialized", "Done");
        telemetry.update();
    }

}
