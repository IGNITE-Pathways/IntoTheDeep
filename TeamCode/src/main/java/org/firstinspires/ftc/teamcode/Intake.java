package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {
    //Horizontal Misumis / INTAKE
    public DcMotor intakeDC = null;;
    private ElapsedTime runtime = new ElapsedTime();
    public Diffy diffy = null;

    public Intake(HardwareMap hardwareMap) {
        intakeDC = hardwareMap.get(DcMotor.class, "intakedc"); //chub 0
        intakeDC.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeDC.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeDC.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeDC.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        diffy = new Diffy(hardwareMap);
    }

    public void initialize() {
        driveToPosition(1, 0, 5);
    }

    private void driveToPosition(double maxSpeed, double inches, double timeoutMilliSeconds) {
        // Ensure that the OpMode is still active
        int newTarget = (int)(inches * XBot.COUNTS_PER_INCH);
        intakeDC.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        intakeDC.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        intakeDC.setPower(Math.abs(maxSpeed));

        // keep looping while we are still active, and there is time left, and intakeDC motor is running.
        // Note: We use (isBusy()) in the loop test, which means that when intakeDC motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        while ((runtime.milliseconds() < timeoutMilliSeconds) && (intakeDC.isBusy())) {
            // Set motor power
            intakeDC.setPower(maxSpeed);
        }
        // Stop all motion;
        intakeDC.setPower(0);
    }

    public int getPosition() {
        return intakeDC.getCurrentPosition();
    }

    public void extendFully() {
        driveToPosition(0.5,6, 5000);
    }

    public void moveToTransferPosition() {
        driveToPosition(0.5,1.5, 5000);
        diffy.moveToTransferPosition();
    }

    public void collapse() {
        driveToPosition(0.5,0, 5000);
    }

    public void closeClaw() {
        diffy.intakeClaw.setPosition(0.5);
    }

    public void openClaw() {
        diffy.intakeClaw.setPosition(1);
    }

    public void moveDiffyToPickPosition() {
        diffy.moveToPickPosition();
    }

    public void rotateDiffy(double degrees) {
        diffy.rotate(degrees);
    }

    public void rotateDiffyUp(double degrees) {
        diffy.rotateUp(degrees);
    }

    public void moveDiffyToNormalPosition() {
        diffy.moveToNormalPosition();
    }
}
