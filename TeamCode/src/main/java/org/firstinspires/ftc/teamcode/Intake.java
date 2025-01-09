package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private PIDFController controller;
    public static double p = 0.03, i = 0, d = 0.0001;
    public static double f = 0.00004;

    public static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!

    //Horizontal Misumis / INTAKE
    public DcMotorEx intakeDCMotor = null;;
    private ElapsedTime runtime = new ElapsedTime();
    public Diffy diffy = null;

    // Store the target in a class-level variable
    public double targetPosition = 0.0;

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
    }

    // 2) Method to call each time in from while loop in DriverControl.runOpMode()
    public void updateOuttakePID() {
        // Read the current position from the motor encoder
        double currentTicks = intakeDCMotor.getCurrentPosition();
        // FTCLib: pass both measurement and the setpoint
        double output = controller.calculate(currentTicks, targetPosition);
        // Set the motors to the calculated power
        intakeDCMotor.setPower(output);
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

    public void moveDiffyToPREPICKPosition() {
        diffy.movetoPREPICKPosition();
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
