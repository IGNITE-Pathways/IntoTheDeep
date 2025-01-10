package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private PIDFController controller;
    public static double p = 0.05, i = 0, d = 0.0005;
    public static double f = 0.00004;

    public static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!

    // The tolerance we allow for the final position
    private static final double POSITION_TOLERANCE = 10; // example: 10 ticks

    //Horizontal Misumis / INTAKE
    public DcMotorEx intakeDCMotor = null;;
    public Diffy diffy = null;

    // Store the target in a class-level variable
    public double targetPosition = 0.0;

    //Circle on Sony (or B on Logitech) toggles between IntakeSlidesPosition.SHORT and IntakeSlidesPosition.FULL
    public IntakeSlidesPosition intakeSlidesPosition = IntakeSlidesPosition.FULL;

    //Cross on Sony (or A on Logitech)  toggles between DiffyVerticalPosition.FLAT and DiffyVerticalPosition.DOWN only while in PICKING_GAME_ELEMENT
    public DiffyVerticalPosition diffyVerticalPosition = DiffyVerticalPosition.FLAT;

    //driver-controlled (Left bumper increments angle by 45° counterclockwise, Right bumper increments angle by 45° clockwise)
    public DiffyHorizontalPosition diffyHorizontalPosition = DiffyHorizontalPosition.ANGLE_0;

    public ClawPosition intakeClawPosition = ClawPosition.OPEN;

    public Intake(HardwareMap hardwareMap) {
        intakeDCMotor = hardwareMap.get(DcMotorEx.class, "intakedc"); //chub 0
        intakeDCMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        diffy = new Diffy(hardwareMap);
        controller = new PIDFController(p, i, d, f);
        controller.setTolerance(5); // optional: how close to setpoint you want to be in ticks
    }

    public void initialize() {
        setPositionInInches(0);
        openClaw();
    }

    // 1) Method to set the PID controller’s setpoint
    public void setPositionInInches(double inches) {
        targetPosition = inches * TICKS_PER_INCH;
        // Run until at setpoint or forced out of loop
//        while (!isAtSetpoint(intakeDCMotor.getCurrentPosition(), targetPosition)) {
//            updateIntakePID();
//            // Let the system keep breathing
//            try {
//                sleep(10);
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }
//
//        // Stop
//        intakeDCMotor.setPower(0);
    }

    private boolean isAtSetpoint(double currentTicks, double targetTicks) {
        return Math.abs(currentTicks - targetTicks) < POSITION_TOLERANCE;
    }

    // 2) Method to call each time in from while loop in DriverControl.runOpMode()
    public void updateIntakePID() {
        if (!isAtSetpoint(intakeDCMotor.getCurrentPosition(), targetPosition)) {
            // Read the current position from the motor encoder
            double currentTicks = intakeDCMotor.getCurrentPosition();
            // FTCLib: pass both measurement and the setpoint
            double output = controller.calculate(currentTicks, targetPosition);
            // Set the motors to the calculated power
            intakeDCMotor.setPower(output);
        } else {
            intakeDCMotor.setPower(0);
        }
    }

    public int getPosition() {
        return intakeDCMotor.getCurrentPosition();
    }

    public void extendLittleBit() {
        setPositionInInches(9);
    }

    public void extendFully() {
        setPositionInInches(8.5);
    }

    public void moveToTransferPosition() {
        setPositionInInches(2);
        diffy.moveToTransferPosition();
    }

    public void InitializePositionOfDiffy() {
        diffy.moveToInitializePosition();
    }

    public void InitializePositionOfDiffyAfterSample() {
        diffy.moveToInitializePositionButAfterIntakingSample();
    }

    public void retractFully() {
        setPositionInInches(0);
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

//    public void moveDiffyToPREPICKPosition() {
//        diffy.movetoPREPICKPosition();
//    }

    public void rotateDiffy(double degrees) {
        diffy.rotate(degrees);
    }

    public void rotateDiffyUp(double degrees) {
        diffy.rotateUp(degrees);
    }

    public void moveDiffyToNormalPosition() {
        diffy.moveToNormalPosition();
    }

    public boolean isClawClosed() {
        return diffy.intakeClaw.getPosition() < 0.4;
    }

    public boolean isClawOpen() {
        return diffy.intakeClaw.getPosition() > 0.9;
    }
}
