package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Diffy;
import org.firstinspires.ftc.teamcode.DiffyPosition;

@TeleOp
@Config
public class DiffyClawTest extends OpMode {
    public Diffy diffy = null;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        diffy = new Diffy(hardwareMap);
    }

    double diffyPosition = 0.5;

    @Override
    public void loop() {


        if (gamepad2.circle) {
            //Make both 1
            diffy.openClaw();
        }

        if (gamepad2.triangle) {
            diffy.closeClaw();
        }

        diffyPosition += gamepad2.left_stick_y / 100;
        diffyPosition = Range.clip(diffyPosition, 0, 1);
        diffy.setClawPosition(diffyPosition);

        telemetry.addData("ClawPos", diffy.getClawPosition());
        telemetry.addData("Color Detected",  diffy.getSampleColor());
        telemetry.update();
    }
}