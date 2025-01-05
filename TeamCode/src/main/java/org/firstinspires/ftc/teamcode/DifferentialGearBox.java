package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DifferentialGearBox {
    private Servo diffyLeft;
    private Servo diffyRight;
    private Servo intakeClaw;

    public DifferentialGearBox(HardwareMap hardwareMap) {
        diffyLeft = hardwareMap.get(Servo.class, "claw");
        diffyRight = hardwareMap.get(Servo.class, "claw");
    }

    public enum Direction {
        CLOCKWISE,
        ANTICLOCKWISE
    }

    public void rotate(Direction direction, int degrees) {
        double targetPosition = degrees / 180.0;

        if (direction == Direction.CLOCKWISE) {
            diffyLeft.setPosition(targetPosition);
        } else if (direction == Direction.ANTICLOCKWISE) {
            diffyLeft.setPosition(1.0 - targetPosition);
        }
    }

    public void rotateUp(int degrees) {
        diffyLeft.setPosition(XBot.BOTH_DIFFYS_TRANSFER_ANGLE);
        diffyRight.setPosition(XBot.BOTH_DIFFYS_TRANSFER_ANGLE);

    }

    public void diffyGetReadyToPickSample() {

        // Ensure the claw is open
        if (Math.abs(intakeClaw.getPosition() - XBot.CLAW_OPEN) > 0.1) {
            intakeClaw.setPosition(XBot.CLAW_OPEN);
        }
        // Move the differential arms to the "pick" position
        diffyLeft.setPosition(XBot.BOTH_DIFFYS_PICK_POSITION);
        diffyRight.setPosition(XBot.BOTH_DIFFYS_PICK_POSITION);

        // Optional: Wait for the servos to reach their target position (non-blocking)
        telemetry.addData("Diffy Status", "Moving to pick position");
        telemetry.update();
    }

    public void diffyPickSample() {
       intakeClaw.setPosition(XBot.CLAW_CLOSE);
    }

    public void diffyDropSample() {
        intakeClaw.setPosition(XBot.CLAW_OPEN);
    }
}