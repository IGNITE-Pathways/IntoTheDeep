package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DifferentialGearBox {
    private Servo diffyLeft;
    private Servo diffyRight;

    public DifferentialGearBox(HardwareMap hardwareMap) {
        diffyLeft = hardwareMap.get(Servo.class, "claw");
        diffyRight = hardwareMap.get(Servo.class, "claw");
    }

    private void initialize() {
        diffyLeft.setPosition(XBot.CLAW_CLOSE);
        diffyRight.setPosition(XBot.CLAW_CLOSE);
    }

    public enum Direction {
        CLOCKWISE,
        ANTICLOCKWISE
    }


    public void rotate(Direction direction, int degrees) {
        //
    }

    public void rotateUp(int degrees) {

    }

    public void diffyDown() {
        diffyLeft.setPosition(XBot.DIFFY_PICK_ANGLE);
        diffyRight.setPosition(XBot.DIFFY_PICK_ANGLE);
        //return Math.abs(claw.getPosition() - XBot.CLAW_OPEN) > 0.1;
    }

}