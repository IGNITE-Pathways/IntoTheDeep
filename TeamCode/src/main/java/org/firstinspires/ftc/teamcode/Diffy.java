package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


public class Diffy {
    public Servo diffyLeft;
    public Servo diffyRight;
    public Servo intakeClaw = null;
    public ColorRangeSensor intakeSensor = null;

    // Current “rotation” angle of the claw about its axis (in degrees).
    public double diffyRotationDegrees = 0;

    // Current “vertical” angle of the claw (in degrees).
    public double diffyVerticalAngle = 0;

    // Tune these maximum angles based on your mechanical limits:
    private static final int MAX_ROTATION_DEGREES = 90;
    private static final int MAX_VERTICAL_DEGREES  = 255;  // e.g. can tilt from 0° up to 180°

    // Used to map degrees to servo range [0.0 ... 1.0].
    // Adjust if servo doesn’t map 0°→0.0 and 180°→1.0 exactly.
    private static final double SERVO_RANGE_DEGREES = 354.5;  // 180 for typical 0–180 servo

    // If mechanical “neutral” is somewhere else, you can change the base offset.
    // For example, 0.5 is center position on a typical servo.
//    private static final double SERVO_CENTER = 0;

    //Cross on Sony (or A on Logitech)  toggles between DiffyVerticalPosition.FLAT and DiffyVerticalPosition.DOWN only while in PICKING_GAME_ELEMENT
    public DiffyVerticalPosition diffyVerticalPosition = DiffyVerticalPosition.FLAT;

    //driver-controlled (Left bumper increments angle by 45° counterclockwise, Right bumper increments angle by 45° clockwise)
    public DiffyHorizontalPosition diffyHorizontalPosition = DiffyHorizontalPosition.ANGLE_0;

    public ClawPosition intakeClawPosition = ClawPosition.OPEN;

    public Diffy(HardwareMap hardwareMap) {
        diffyLeft = hardwareMap.get(Servo.class, "diffyleft");
        diffyRight = hardwareMap.get(Servo.class, "diffyright");
        diffyLeft.setDirection(Servo.Direction.REVERSE);
        intakeClaw = hardwareMap.get(Servo.class, "intakeclaw"); // ehub 4
        intakeSensor = hardwareMap.get(ColorRangeSensor.class, "intakesensor"); // ehub I2C 0
    }

    public void initialize() {

    }

    /**
     * Rotate the claw about its axis by `degrees` in the specified direction.
     * Positive degrees for one direction, negative for the opposite.
     */
    public void rotate(Direction direction, int degrees) {
        // Update diffyDegrees by +/- degrees
        if (direction == Direction.CLOCKWISE) {
            diffyRotationDegrees += degrees;
        } else {
            diffyRotationDegrees -= degrees;
        }

        // Clamp if desired (just to prevent going beyond mechanical limits):
        diffyRotationDegrees = Range.clip(diffyRotationDegrees, -MAX_ROTATION_DEGREES, MAX_ROTATION_DEGREES);

        // Update the servos based on new angles
        updateServos();
    }


    public void setDiffyRotationDegrees(double degrees) {
        // Update diffyDegrees by +/- degrees
        diffyRotationDegrees = degrees;

        // Clamp if desired (just to prevent going beyond mechanical limits):
        diffyRotationDegrees = Range.clip(diffyRotationDegrees, -MAX_ROTATION_DEGREES, MAX_ROTATION_DEGREES);

        // Update the servos based on new angles
        updateServos();
    }
    /**
     * Rotate the claw mechanism by `degrees`.
     * If we want to move counterclockwise, we can pass negative degrees
     */
    public void rotate(double degrees) {
        diffyRotationDegrees += degrees;

        // Clamp if desired
        diffyRotationDegrees = Range.clip(diffyRotationDegrees, -MAX_ROTATION_DEGREES, MAX_ROTATION_DEGREES);

        // Update the servos based on new angles
        updateServos();
    }

    /**
     * Rotate the entire differential mechanism upward by `degrees`.
     * If we want to move downward, we can pass negative degrees
     */
    public void rotateUp(double degrees) {
        diffyVerticalAngle += degrees;

        // Clamp if desired
        diffyVerticalAngle = Range.clip(diffyVerticalAngle, 0, MAX_VERTICAL_DEGREES);

        // Update the servos based on new angles
        updateServos();
    }

    public void setDiffyVerticalAngle(double degrees) {
        diffyVerticalAngle = degrees;

        // Clamp if desired
        diffyVerticalAngle = Range.clip(diffyVerticalAngle, 0, MAX_VERTICAL_DEGREES);

        // Update the servos based on new angles
        updateServos();
    }

    /**
     * This is the key method that calculates servo positions based on:
     *  - The ‘vertical’ lift angle (same direction movement)
     *  - The ‘rotation’ angle (opposite direction movement)
     */
    private void updateServos() {
        // Convert angles (in degrees) to a fractional offset [–1 ... +1]
        // relative to some reference. For example:
        double rotationFraction = (diffyRotationDegrees) / SERVO_RANGE_DEGREES;         // –1 to +1 range if ±180
        double verticalFraction = diffyVerticalAngle / SERVO_RANGE_DEGREES;   // 0 to ~1 if 0–180

        /*
         * Because we have a “differential”:
         * - Both servos moving in the SAME direction => “vertical” lift
         * - Both servos moving in OPPOSITE directions => “rotation”
         *
         * Suppose we want:
         *   leftServo = CENTER + verticalFraction + rotationFraction
         *   rightServo= CENTER + verticalFraction – rotationFraction
         *
         * The +rotationFraction and –rotationFraction produces opposite directions for rotation.
         * The +verticalFraction on both produces the “same direction” movement for lifting.
         *
         * Then we clamp to [0, 1].
         */

        double leftPos  = verticalFraction + rotationFraction;
        double rightPos = verticalFraction - rotationFraction;

        // Make sure we don’t go beyond servo limits
        leftPos  = Range.clip(leftPos,  0.0, 1.0);
        rightPos = Range.clip(rightPos, 0.0, 1.0);

        // Finally set the servo positions
        diffyLeft.setPosition(leftPos);
        diffyRight.setPosition(rightPos);
    }

//    public void setDiffyPositions(double leftPos, double rightPos) {
//        diffyLeft.setPosition(leftPos);
//        diffyRight.setPosition(rightPos);
//    }

//    public void moveToPickPosition() {
//        diffyRotationDegrees = 0;
//        diffyVerticalAngle = 0;
//        updateServos();
//        //Open Claw
//        intakeClaw.setPosition(1);
//    }

//    public void moveToInitializePosition() {
//        diffyRotationDegrees = 0;
//        diffyVerticalAngle = 215;
//        updateServos();
//        //Open Claw
//        intakeClaw.setPosition(1);
//    }

    public SampleColor getSampleColor() {
        if (intakeSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
            // Cast, then enable or disable the LED
            ((com.qualcomm.robotcore.hardware.SwitchableLight) intakeSensor).enableLight(true);  // turn on
        }
        // Read raw RGB values
        int redValue   = intakeSensor.red();
        int greenValue = intakeSensor.green();
        int blueValue  = intakeSensor.blue();

        SampleColor detectedColor = SampleColor.UNKNOWN;
        if (redValue > 100 && greenValue > 100 && blueValue < 100) {
            detectedColor = SampleColor.YELLOW;
        } else if (blueValue > redValue && blueValue > greenValue) {
            detectedColor = SampleColor.BLUE;
        } else if (redValue > blueValue && redValue > greenValue) {
            detectedColor = SampleColor.RED;
        }

        if (intakeSensor instanceof com.qualcomm.robotcore.hardware.SwitchableLight) {
            // Cast, then enable or disable the LED
            ((com.qualcomm.robotcore.hardware.SwitchableLight) intakeSensor).enableLight(false);  // turn on
        }
        return detectedColor;
    }

}