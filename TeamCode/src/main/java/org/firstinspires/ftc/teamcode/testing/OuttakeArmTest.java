package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.OuttakeArmPosition;
import org.firstinspires.ftc.teamcode.OuttakeSlidesPosition;

@TeleOp
@Config
public class OuttakeArmTest extends OpMode {

    Outtake outtake = null;
    public static double outtakeArmAngle = 0;

    public static OuttakeSlidesPosition outtakeSlidesPosition = OuttakeSlidesPosition.PRE_TRANSFER;

    public static boolean clawOpen = true;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        outtake = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {
        outtake.setOuttakeArmAngle(outtakeArmAngle);
        outtake.setOuttakeSlidesPositionSync(outtakeSlidesPosition);
        if (clawOpen) outtake.openClaw();
        else outtake.closeClaw();

        telemetry.addData("Arm Angle", outtake.outtakeArmAngle);
        telemetry.addData("Arm Left Pos", outtake.getLeftArmPosition());
        telemetry.addData("Arm Right Pos", outtake.getRightArmPosition());
        telemetry.addData("Slides Position in Ticks", outtake.targetSlidesPosition);
        telemetry.update();
    }
}