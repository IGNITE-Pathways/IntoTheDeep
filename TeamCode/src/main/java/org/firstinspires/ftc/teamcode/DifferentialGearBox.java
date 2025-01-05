package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.annotation.SuppressLint;
import android.service.credentials.Action;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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

    public enum DIRECTION {
        CLOCKWISE,
        ANTICLOCKWISE
    }

    public enum DEGREES {
        DEGREES_45(45),
        DEGREES_90(90),
        DEGREES_135(135),
        DEGREES_180(180),
        DEGREES_225(225),
        DEGREES_270(270),
        DEGREES_315(315);

        private final int value;

        DEGREES(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
    public void rotate(DIRECTION direction, DEGREES degrees) {
        //
    }

    public void rotateUp(DEGREES degrees) {

    }

    public void diffyDown() {
        diffyLeft.setPosition(XBot.DIFFY_PICK_ANGLE);
        diffyRight.setPosition(XBot.DIFFY_PICK_ANGLE);
        //return Math.abs(claw.getPosition() - XBot.CLAW_OPEN) > 0.1;
    }

}