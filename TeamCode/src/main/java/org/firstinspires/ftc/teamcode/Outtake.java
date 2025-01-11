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

    private static final double SERVO_RANGE_DEGREES = 255;  // 180 for typical 0–180 servo
    public static final int MAX_ROTATION_DEGREES = 255;

    // Example: 537.7 ticks/rev, a wheel/spool diameter = 1.377" => circumference ~4.33"
    // double TICKS_PER_REV = 537.7; // Example
    // double WHEEL_CIRCUMFERENCE = Math.PI * 1.377;  // example spool diameter
    // double TICKS_PER_INCH = TICKS_PER_REV / WHEEL_CIRCUMFERENCE;
    private static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!

    // The tolerance we allow for the final position
    private static final double POSITION_TOLERANCE = 10; // example: 10 ticks

    // Store the target in a class-level variable, in ticks
    public double targetSlidesPosition = 0.0;

    private OuttakeSlidesPosition outtakeSlidesPosition = OuttakeSlidesPosition.CLOSE;

    //OuttakeArmPosition changes automatically based on current game element and Game State
    private OuttakeArmPosition outtakeArmPosition = OuttakeArmPosition.FACING_DOWN;
    public double outtakeArmAngle = 0;
    private static final double ARM_ANGLE_TOLERANCE = 1; // in degrees

    private ClawPosition outtakeClawPosition = ClawPosition.OPEN;
    boolean usePID = false;

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

        if (!usePID) {
            outtakeDCLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeDCLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeDCLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            outtakeDCRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            outtakeDCRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            outtakeDCRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            controller = new PIDFController(p, i, d, f);
            controller.setTolerance(5); // optional: how close to setpoint you want to be in ticks
        }
    }

    public void initialize() {
        setOuttakeArmPosition(OuttakeArmPosition.FACING_DOWN);
        openClaw();
        setPositionInInchesSync(0);
    }

    private void setPositionInInches(double inches) {
        targetSlidesPosition = inches * TICKS_PER_INCH;
    }

    public boolean areSlidesAtPosition() {
        return isAtSetpoint(outtakeDCLeft.getCurrentPosition(), targetSlidesPosition);
    }

    public void setPositionInInchesSync(double inches) {
        targetSlidesPosition = inches * TICKS_PER_INCH;
        // Run until at setpoint or forced out of loop
        while (!isAtSetpoint(outtakeDCLeft.getCurrentPosition(), targetSlidesPosition)) {
            double current = outtakeDCLeft.getCurrentPosition();
            double output = controller.calculate(current, targetSlidesPosition);
            outtakeDCLeft.setPower(output);
            outtakeDCRight.setPower(output);

            // Let the system keep breathing
            try {
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        // Stop
        outtakeDCLeft.setPower(0);
        outtakeDCRight.setPower(0);
    }

    // 2) Method to call each time in from while loop in DriverControl.runOpMode()
    public void updateOuttakePID() {
        if (!isAtSetpoint(outtakeDCLeft.getCurrentPosition(), targetSlidesPosition)) {
            // Read the current position from the motor encoder
            double currentTicks = outtakeDCRight.getCurrentPosition();
            // FTCLib: pass both measurement and the setpoint
            double output = controller.calculate(currentTicks, targetSlidesPosition);
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

    public int getLeftPosition() {
        return outtakeDCLeft.getCurrentPosition();
    }

    public int getRightPosition() {
        return outtakeDCRight.getCurrentPosition();
    }

    public void setOuttakeArmAngle(double degrees) {
        outtakeArmAngle = degrees;
        outtakeArmAngle = Range.clip(outtakeArmAngle, 0, MAX_ROTATION_DEGREES);
        // Update the servos based on new angles
        updateServos();
    }

    private void updateServos() {
        // Convert angles (in degrees) to a fractional offset [–1 ... +1]
        // relative to some reference. For example:
        double pos = outtakeArmAngle / SERVO_RANGE_DEGREES;   // 0 to ~1 if 0–180

        // Make sure we don’t go beyond servo limits
        pos = Range.clip(pos, 0.0, 1.0);

        // Finally set the servo positions
        outtakeServoLeft.setPosition(pos);
        outtakeServoRight.setPosition(pos);
    }

    //Actual Position in degrees
    private double getOuttakeArmPosition() {
        return  outtakeServoLeft.getPosition() * SERVO_RANGE_DEGREES;
    }

    //Desired position in degrees
    private double getOuttakeArmTargetAngle() {
        return outtakeArmPosition.getDegrees();
    }

    private boolean isArmAtTargetPosition() {
        return Math.abs(getOuttakeArmTargetAngle() - getOuttakeArmPosition()) < ARM_ANGLE_TOLERANCE;
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

    public boolean isClawClosed() {
        return outtakeClaw.getPosition() < 0.4;
    }
    public boolean isClawOpen() {
        return outtakeClaw.getPosition() > 0.9;
    }

    //Called from, DriverControl:runOpMode
    public void loop() {
        if (usePID) {
            updateOuttakePID();
        }
    }

    public void setOuttakeSlidesPosition(OuttakeSlidesPosition position) {
        this.outtakeSlidesPosition = position;
        setPositionInInches(position.getPosition());
        if (!isAtSetpoint(outtakeDCLeft.getCurrentPosition(), targetSlidesPosition) && !usePID) {
            driveToPosition(1, position.getPosition(), 10000);
        }
    }

    public void setOuttakeSlidesPositionSync(OuttakeSlidesPosition outtakeSlidesPosition) {
        setOuttakeSlidesPosition(outtakeSlidesPosition);
        if (!isAtSetpoint(outtakeDCLeft.getCurrentPosition(), targetSlidesPosition)) {
            if (usePID) {
                setPositionInInchesSync(outtakeSlidesPosition.getPosition());
            } else {
                driveToPosition(1, outtakeSlidesPosition.getPosition(), 10000); //@ToDo
            }
        }
    }

    public void setOuttakeClawPosition(ClawPosition position) {
        this.outtakeClawPosition = position;
        switch (outtakeClawPosition) {
            case CLOSE:
                if (!isClawClosed()) closeClaw();
                break;
            case OPEN:
                if (!isClawOpen()) openClaw();
                break;
        }
    }

    public void setOuttakeArmPosition(OuttakeArmPosition position) {
        this.outtakeArmPosition = position;
        setOuttakeArmAngle(this.outtakeArmPosition.getDegrees());
    }

    //If not using PID
    private void driveToPosition(double maxSpeed, double inches, double timeoutMilliSeconds) {
        // Ensure that the OpMode is still active
        int newTarget = (int)(inches * TICKS_PER_INCH);
        outtakeDCLeft.setTargetPosition(newTarget);
        outtakeDCRight.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        outtakeDCLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outtakeDCRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        outtakeDCLeft.setPower(Math.abs(maxSpeed));
        outtakeDCRight.setPower(Math.abs(maxSpeed));

        // keep looping while we are still active, and there is time left, and outtakeDC motors are running.
        // Note: We use (isBusy()) in the loop test, which means that when outtakeDC motors hits
        // their target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        while ((runtime.milliseconds() < timeoutMilliSeconds) && (outtakeDCLeft.isBusy())) {
            // Set motor power
            outtakeDCLeft.setPower(maxSpeed);
            outtakeDCRight.setPower(maxSpeed);
        }
        // Stop all motion;
        outtakeDCLeft.setPower(0);
        outtakeDCRight.setPower(0);
    }

}
