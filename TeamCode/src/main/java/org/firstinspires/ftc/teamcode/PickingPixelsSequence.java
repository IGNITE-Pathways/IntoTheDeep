package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Picking Pixels Sequence", group = "Linear Opmode")
public class PickingPixelsSequence extends LinearOpMode {

    // Constants for servo position control
    private static final double SERVO_INCREMENT = 0.05; // Increment size for position changes
    private static final double SERVO_MIN = 0.0;        // Minimum servo position
    private static final double SERVO_MAX = 1.0;        // Maximum servo position
    private static final double ELBOW_TARGET_POSITION = 0.3; // Specific position for the elbow servo

    // Servo objects
    private Servo extendo;
    private Servo elbow;
    private Servo roller;

    // Servo positions
    private double extendoPosition = 0.0; // Start fully retracted
    private double rollerPosition = 0.0; // Start roller off

    // State control for "Picking Pixels"
    private enum State {
        PICKING_SAMPLES
    }
    private State currentState = null; // Default no state

    @Override
    public void runOpMode() {
        // Initialize hardware
        extendo = hardwareMap.get(Servo.class, "extendo");
        elbow = hardwareMap.get(Servo.class, "elbow");
        roller = hardwareMap.get(Servo.class, "roller");

        // Set initial positions
        extendo.setPosition(extendoPosition);
        elbow.setPosition(0.5); // Neutral position
        roller.setPosition(rollerPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the start button
        waitForStart();

        while (opModeIsActive()) {
            // Trigger the sequence by pressing the "X" button if not already active
            if (gamepad2.x && currentState == null) {
                currentState = State.PICKING_SAMPLES; // Start the sequence
            }

            // State machine for "Picking Pixels" sequence
            if (currentState != null) {
                switch (currentState) {
                    case PICKING_SAMPLES:
                        extendoPosition += SERVO_INCREMENT;
                        if (extendoPosition >= SERVO_MAX) {
                            extendoPosition = SERVO_MAX; // Clamp to max

                        }
                        extendo.setPosition(extendoPosition);
                        elbow.setPosition(ELBOW_TARGET_POSITION); // Set elbow to 0.3 directly
                        rollerPosition = 1.0; // Start rolling
                        roller.setPosition(rollerPosition);
                        // Stay in this state; the roller continues indefinitely
                        break;
                }
            }

            // Telemetry for debugging
            telemetry.addData("Current State", currentState != null ? currentState : "Not Active");
            telemetry.addData("Extendo Position", extendoPosition);
            telemetry.addData("Elbow Position", elbow.getPosition());
            telemetry.addData("Roller Position", rollerPosition);
            telemetry.update();
        }
    }
}
