package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Outtake {
    private final ElapsedTime runtime = new ElapsedTime();

    //Vertical Misumis / OUTTAKE
    private DcMotor outtakeDCRight = null;
    private DcMotor outtakeDCLeft = null;
    private Servo outtakeServoRight = null;
    private Servo outtakeServoLeft = null;
    private Servo outtakeClaw = null;

    public double outtakeAngle = 0;

    private static final int MAX_ROTATION_DEGREES = 255;
    private static final double SERVO_RANGE_DEGREES = 255;  // 180 for typical 0–180 servo

    public Outtake(HardwareMap hardwareMap) {
        outtakeDCRight = hardwareMap.get(DcMotor.class, "outtakedcright"); //chub 1
        outtakeDCLeft = hardwareMap.get(DcMotor.class, "outtakedcleft"); //ehub 2

        //Initialize the Servo variables
        outtakeServoRight = hardwareMap.get(Servo.class, "outtakeservoright"); // chub 5
        outtakeServoLeft = hardwareMap.get(Servo.class, "outtakeservoleft"); // ehub 0
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeclaw"); // ehub 1

        outtakeDCRight.setDirection(DcMotor.Direction.FORWARD);
        outtakeDCLeft.setDirection(DcMotor.Direction.REVERSE);

        outtakeDCRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeDCLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeDCRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeDCRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeDCLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeDCLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeServoLeft.setDirection(Servo.Direction.FORWARD);
        outtakeServoRight.setDirection(Servo.Direction.REVERSE);
    }

    public void initialize() {
        driveToPosition(1, 0, 5);
        rotateArmDown();
    }

    private void driveToPosition(double maxSpeed, double inches, double timeoutMilliSeconds) {
        // Ensure that the OpMode is still active
        int newTarget = (int) (inches * XBot.COUNTS_PER_INCH);
        outtakeDCLeft.setTargetPosition(newTarget);
        outtakeDCRight.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        outtakeDCLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeDCRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        outtakeDCLeft.setPower(Math.abs(maxSpeed));
        outtakeDCRight.setPower(Math.abs(maxSpeed));

        // keep looping while we are still active, and there is time left, and intakeDC motor is running.
        // Note: We use (isBusy()) in the loop test, which means that when intakeDC motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        while ((runtime.milliseconds() < timeoutMilliSeconds) && (outtakeDCLeft.isBusy())) {
            // Set motor power
            outtakeDCLeft.setPower(maxSpeed);
            outtakeDCRight.setPower(maxSpeed);
        }
        // Stop all motion;
        outtakeDCLeft.setPower(0.025);
        outtakeDCRight.setPower(0.025);
    }


    public void moveToSampleDropPosition() {
        driveToPosition(0.5, 25, 2000);
    }

    public void moveToSpecimenDropPosition() {
        driveToPosition(0.5, 5, 2000);
    }

    public void moveToTransferPosition() {
        driveToPosition(0.5, 2.4, 2000);
    }

    public void collapse() {
        driveToPosition(0.5, 0, 2000);
    }

    public int getLeftPosition() {
        return outtakeDCLeft.getCurrentPosition();
    }

    public int getRightPosition() {
        return outtakeDCRight.getCurrentPosition();
    }

    public void rotateArmToTransferPosition() {
        rotate(0);
    }

    public void rotateArmDown() {
        rotate(70);
    }

    public void rotateArmToSampleDropPosition() {
        rotate(SERVO_RANGE_DEGREES); //Max rotate
    }

    public void rotate(double degrees) {
        outtakeAngle = degrees;
        outtakeAngle = Range.clip(outtakeAngle, 0, MAX_ROTATION_DEGREES);
        // Update the servos based on new angles
        updateServos();
    }

    private void updateServos() {
        // Convert angles (in degrees) to a fractional offset [–1 ... +1]
        // relative to some reference. For example:
        double pos = outtakeAngle / SERVO_RANGE_DEGREES;   // 0 to ~1 if 0–180

        // Make sure we don’t go beyond servo limits
        pos  = Range.clip(pos,  0.0, 1.0);

        // Finally set the servo positions
        outtakeServoLeft.setPosition(pos);
        outtakeServoRight.setPosition(pos);
    }

    public void rotateServosDirectly(double pos) {
        outtakeServoLeft.setPosition(pos);
        outtakeServoRight.setPosition(pos);
    }

    public void openClaw() {
        outtakeClaw.setPosition(1);
    }

    public void closeClaw() {
        outtakeClaw.setPosition(0);
    }

    public double getOuttakeLeftServoPosition() {
        return outtakeServoLeft.getPosition();
    }

    public double getOuttakeRightServoPosition() {
        return outtakeServoRight.getPosition();
    }
}
