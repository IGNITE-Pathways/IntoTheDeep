package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.arcrobotics.ftclib.controller.PIDFController;

public class Outtake {

    private PIDFController controller;
    public static double p = 0.03, i = 0, d = 0.0001;
    public static double f = 0.00004;

    private final ElapsedTime runtime = new ElapsedTime();

    //Vertical Misumis / OUTTAKE
    private DcMotorEx outtakeDCRight = null;
    private DcMotorEx outtakeDCLeft = null;
    private Servo outtakeServoRight = null;
    private Servo outtakeServoLeft = null;
    private Servo outtakeClaw = null;

    public double outtakeAngle = 0;

    private static final int MAX_ROTATION_DEGREES = 255;
    private static final double SERVO_RANGE_DEGREES = 255;  // 180 for typical 0–180 servo

    // Example: 537.7 ticks/rev, a wheel/spool diameter = 1.377" => circumference ~4.33"
    // double TICKS_PER_REV = 537.7; // Example
    // double WHEEL_CIRCUMFERENCE = Math.PI * 1.377;  // example spool diameter
    // double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;
    public static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!

    // The tolerance we allow for the final position
    private static final double POSITION_TOLERANCE = 10; // example: 10 ticks

    // Store the target in a class-level variable
    public double targetPosition = 0.0;

    public Outtake(HardwareMap hardwareMap) {
        outtakeDCRight = hardwareMap.get(DcMotorEx.class, "outtakedcright"); //chub 1
        outtakeDCLeft = hardwareMap.get(DcMotorEx.class, "outtakedcleft"); //ehub 2

        //Initialize the Servo variables
        outtakeServoRight = hardwareMap.get(Servo.class, "outtakeservoright"); // chub 5
        outtakeServoLeft = hardwareMap.get(Servo.class, "outtakeservoleft"); // ehub 0
        outtakeClaw = hardwareMap.get(Servo.class, "outtakeclaw"); // ehub 1

        outtakeDCRight.setDirection(DcMotor.Direction.FORWARD);
        outtakeDCLeft.setDirection(DcMotor.Direction.REVERSE);

        outtakeServoLeft.setDirection(Servo.Direction.FORWARD);
        outtakeServoRight.setDirection(Servo.Direction.REVERSE);

        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(5); // optional: how close to setpoint you want to be in ticks
    }

    public void initialize() {
        rotateArmDown();
        setPositionInInches(0);
    }

    // 1) Method to set the PID controller’s setpoint
    public void setPositionInInches(double inches) {
        targetPosition = inches * TICKS_PER_INCH;
//        // Run until at setpoint or forced out of loop
//        while (!isAtSetpoint(outtakeDCLeft.getCurrentPosition(), targetPosition)) {
//            double current = outtakeDCLeft.getCurrentPosition();
//            double output = controller.calculate(current, targetPosition);
//            outtakeDCLeft.setPower(output);
//            outtakeDCRight.setPower(output);
//
//            // Let the system keep breathing
//            try {
//                sleep(10);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }
//        // Stop
//        outtakeDCLeft.setPower(0);
//        outtakeDCRight.setPower(0);
    }

    // 2) Method to call each time in from while loop in DriverControl.runOpMode()
    public void updateOuttakePID() {
        if (!isAtSetpoint(outtakeDCLeft.getCurrentPosition(), targetPosition)) {
            // Read the current position from the motor encoder
            double currentTicks = outtakeDCRight.getCurrentPosition();
            // FTCLib: pass both measurement and the setpoint
            double output = controller.calculate(currentTicks, targetPosition);
            // Set the motors to the calculated power
            outtakeDCRight.setPower(output);
            outtakeDCLeft.setPower(output);
        } else {
            outtakeDCLeft.setPower(0);
            outtakeDCRight.setPower(0);
        }
    }

    private boolean isAtSetpoint(double currentTicks, double targetTicks) {
        return Math.abs(currentTicks - targetTicks) < POSITION_TOLERANCE;
    }

    public void moveToSampleDropPosition() {
        setPositionInInches(10);
    }

    public void moveToSpecimenDropPosition() {
        setPositionInInches(5);
    }

    public void moveToTransferPosition() {
        setPositionInInches(2.4);
    }

    public void collapse() {
        setPositionInInches(0);
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
        pos = Range.clip(pos, 0.0, 1.0);

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
