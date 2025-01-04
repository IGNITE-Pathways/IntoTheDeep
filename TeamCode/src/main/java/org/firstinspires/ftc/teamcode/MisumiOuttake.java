package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MisumiOuttake {

    private DcMotor outtakeLeft = null;
    private DcMotor outtakeRight = null;
    private Servo gearLeft = null;
    private Servo gearRight = null;
    private Servo Claw = null;
    private ElapsedTime runtime = new ElapsedTime();

    public MisumiOuttake(HardwareMap hardwareMap) {
        outtakeLeft = hardwareMap.get(DcMotor.class, "viper");
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);
        outtakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeRight = hardwareMap.get(DcMotor.class, "viper");
        outtakeRight.setDirection(DcMotor.Direction.REVERSE);
        outtakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtakeRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gearLeft = hardwareMap.get(Servo.class, "claw");

        gearRight = hardwareMap.get(Servo.class, "claw");

        Claw = hardwareMap.get(Servo.class, "claw");

    }

    public void initialize() {
        Claw.setPosition(XBot.CLAW_CLOSE);
    }
}

