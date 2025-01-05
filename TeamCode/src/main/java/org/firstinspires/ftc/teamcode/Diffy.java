package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Diffy {
    private Servo diffyLeft;
    private Servo diffyRight;

    // Current “rotation” angle of the claw about its axis (in degrees).
    // 0 = claw parallel to the robot, positive angles mean rotating in one direction, negative in the other.
    private int diffyDegrees = 0;

    // Current “vertical” angle of the claw (in degrees).
    // 0 = claw pointing downward, positive angles mean lifting upward.
    private int diffyVerticalAngle = 0;

    // Tune these maximum angles based on your mechanical limits:
    private static final int MAX_ROTATION_DEGREES = 180;   // e.g. +/− 90 each way, or something else
    private static final int MAX_VERTICAL_DEGREES  = 225;  // e.g. can tilt from 0° up to 180°

    // Used to map degrees to servo range [0.0 ... 1.0].
    // Adjust if servo doesn’t map 0°→0.0 and 180°→1.0 exactly.
    private static final double SERVO_RANGE_DEGREES = 355.0;  // 180 for typical 0–180 servo

    // If mechanical “neutral” is somewhere else, you can change the base offset.
    // For example, 0.5 is center position on a typical servo.
    private static final double SERVO_CENTER = 0.5;


    public Diffy(HardwareMap hardwareMap) {
        diffyLeft = hardwareMap.get(Servo.class, "diffyleft");
        diffyRight = hardwareMap.get(Servo.class, "diffyright");
        initialize();
    }

    private void initialize() {
        diffyVerticalAngle = 0; // claw pointing downward
        diffyDegrees = 0;       // no rotation
        updateServos();
    }

    public enum Direction {
        CLOCKWISE,
        ANTICLOCKWISE
    }

    /**
     * Rotate the claw about its axis by `degrees` in the specified direction.
     * Positive degrees for one direction, negative for the opposite.
     */
    public void rotate(Direction direction, int degrees) {
        // Update diffyDegrees by +/- degrees
        if (direction == Direction.CLOCKWISE) {
            diffyDegrees += degrees;
        } else {
            diffyDegrees -= degrees;
        }

        // Clamp if desired (just to prevent going beyond mechanical limits):
        diffyDegrees = Range.clip(diffyDegrees, -MAX_ROTATION_DEGREES, MAX_ROTATION_DEGREES);

        // Update the servos based on new angles
        updateServos();
    }

    /**
     * Rotate the claw mechanism by `degrees`.
     * If we want to move counterclockwise, we can pass negative degrees
     */
    public void rotate(int degrees) {
        diffyDegrees += degrees;

        // Clamp if desired
        diffyDegrees = Range.clip(diffyDegrees, -MAX_ROTATION_DEGREES, MAX_ROTATION_DEGREES);

        // Update the servos based on new angles
        updateServos();
    }

    /**
     * Rotate the entire differential mechanism upward by `degrees`.
     * If we want to move downward, we can pass negative degrees
     */
    public void rotateUp(int degrees) {
        diffyVerticalAngle += degrees;

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
        double rotationFraction = diffyDegrees / SERVO_RANGE_DEGREES;         // –1 to +1 range if ±180
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

        double leftPos  = SERVO_CENTER + verticalFraction + rotationFraction;
        double rightPos = SERVO_CENTER + verticalFraction - rotationFraction;

        // Make sure we don’t go beyond servo limits
        leftPos  = Range.clip(leftPos,  0.0, 1.0);
        rightPos = Range.clip(rightPos, 0.0, 1.0);

        // Finally set the servo positions
        diffyLeft.setPosition(leftPos);
        diffyRight.setPosition(rightPos);
    }

}