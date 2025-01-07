package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDFController;

public class Outtake {
    private final ElapsedTime runtime = new ElapsedTime();

    //Vertical Misumis / OUTTAKE
    private DcMotor outtakeDCRight = null;
    private DcMotor outtakeDCLeft = null;
    private Servo outtakeServoRight = null;
    private Servo outtakeServoLeft = null;
    private Servo outtakeClaw = null;

    public int outtakeAngle = 0;

    private static final int MAX_ROTATION_DEGREES = 90;
    private static final double SERVO_RANGE_DEGREES = 255;  // 180 for typical 0–180 servo
    private PIDFController controller;
    public static double p, i, d, f = 0;

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

        controller = new PIDFController(p, i, d, f);
    }

    public void initialize() {
        driveToPosition(0);
    }

    private void driveToPosition(double inches) {
       double currentPosition = outtakeDCLeft.getCurrentPosition();
       controller.setPIDF(p, i, d, f);
       outtakeDCLeft.setPower(controller.calculate(inches));
       outtakeDCRight.setPower(controller.calculate(inches));
    }


    public void moveToSampleDropPosition() { driveToPosition(10); }

    public void moveToSpecimenDropPosition() {
        driveToPosition(5);
    }

    public void moveToTransferPosition() {
        driveToPosition(1.4);
    }

    public void collapse() {
        driveToPosition(0);
    }

    public int getLeftPosition() {
        return outtakeDCLeft.getCurrentPosition();
    }

    public int getRightPosition() {
        return outtakeDCRight.getCurrentPosition();
    }

    public void rotateArmToTransferPosition() {
        rotate(6);
    }

    public void rotateArmToSampleDropPosition() {
        rotate(-45);
    }

    public void rotate(double degrees) {
        outtakeAngle += degrees;

        // Clamp if desired
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
