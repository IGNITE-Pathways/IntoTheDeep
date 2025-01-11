package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Diffy;
import org.firstinspires.ftc.teamcode.DiffyPosition;

@TeleOp
@Config
public class DiffyTest extends OpMode {
    public Diffy diffy = null;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        diffy = new Diffy(hardwareMap);
    }

    double leftPos = DiffyPosition.FLAT.getLeftPos();
    double rightPos = DiffyPosition.FLAT.getRightPos();

    @Override
    public void loop() {

        if (gamepad2.circle) {
            //Make both 1
            diffy.setDiffyPosition(DiffyPosition.FLAT);
        }

        if (gamepad2.triangle) {
            diffy.setDiffyPosition(DiffyPosition.DOWN_PARALLEL);
        }

        if (gamepad2.square) {
            diffy.setDiffyPosition(DiffyPosition.TRANSFER_SAMPLE);
        }

        if (gamepad2.cross) {
            diffy.setDiffyPosition(DiffyPosition.TRANSFER_SPECIMEN);
        }

        if (gamepad2.left_bumper) {
            diffy.setDiffyPosition(DiffyPosition.DOWN_ANTI_CLOCKWISE);
        } else if (gamepad2.right_bumper) {
            diffy.setDiffyPosition(DiffyPosition.DOWN_CLOCKWISE);
        }

        leftPos += gamepad2.left_stick_y / 100;
        rightPos += gamepad2.right_stick_y / 100;

        leftPos = Range.clip(leftPos, 0, 1);
        rightPos = Range.clip(rightPos, 0, 1);

        diffy.setDiffyPositions(leftPos, rightPos);

        telemetry.addData("leftPos:", "%4.2f, rightPos: %4.2f", leftPos, rightPos);
        telemetry.addData("Diffy Position, Left:", "%4.2f, Right: %4.2f", diffy.diffyLeft.getPosition(), diffy.diffyRight.getPosition());
        telemetry.update();
    }
}