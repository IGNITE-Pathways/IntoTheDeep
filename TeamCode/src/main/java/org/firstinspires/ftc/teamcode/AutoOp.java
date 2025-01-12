package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Op", group = "Linear OpMode")
public class AutoOp extends LinearOpMode {

    protected final ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    Intake intake = null;
    Outtake outtake = null;

    @Override
    public void runOpMode() {

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront"); //ehub 0
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftback"); //ehub 1
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront"); //chub 2
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback"); //chub 3 //Encoder used for ODO Y

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        outtake.setOuttakeArmPosition(OuttakeArmPosition.SPECIMEN_DROP);
        outtake.closeClaw();
        outtake.setOuttakeClawPosition(ClawPosition.CLOSE);

        // Telemetry
        telemetry.addData("Arm Left Pos", outtake.getLeftArmPosition());
        telemetry.addData("Arm Right Pos", outtake.getRightArmPosition());
        telemetry.addData("Claw Pos", outtake.getClawPosition());
        telemetry.addData("Status", "Initialized");

        // Show the elapsed game time and wheel power.
        telemetry.update();

        waitForStart();
        runtime.reset();

        outtake.setOuttakeSlidesPositionSync(OuttakeSlidesPosition.HOOK_SPECIMEN_TOP_RUNG);
        sleep(1000);

        moveForward(-.5, 800);

        //Move Slides to Drop
        outtake.setOuttakeArmPosition(OuttakeArmPosition.POST_SPECIMEN_DROP);
        sleep(100);
        //If arm in position
        outtake.setOuttakeSlidesPosition(OuttakeSlidesPosition.POST_HOOK);
        sleep(200);

        outtake.setOuttakeClawPosition(ClawPosition.OPEN);
        sleep(400);

        //Initialization Positions
        outtake.setOuttakeArmPosition(OuttakeArmPosition.FACING_DOWN);
        sleep(400);

        outtake.setOuttakeSlidesPositionSync(OuttakeSlidesPosition.CLOSE);
        sleep(1000);

        moveForward(.5, 800);
        sleep(100);
        strafeRight(.8, 1000);

         // Telemetry
        telemetry.addData("Time Used", runtime.seconds());

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

    private void moveForward(double speed, int milliSeconds) {
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);

        sleep(milliSeconds);

        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void strafeLeft(double speed, int milliSeconds) {
        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);

        sleep(milliSeconds);

        // Stop all motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    private void strafeRight(double speed, int milliSeconds) {
        //Spinning outward
        leftFrontDrive.setPower(speed * .95); //Adjust for bad strafing
        leftBackDrive.setPower(-speed);

        //Spinning inward
        rightFrontDrive.setPower(-speed);
        rightBackDrive.setPower(speed);

        sleep(milliSeconds);

        // Stop all motors
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

}



