package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private PIDFController controller;
    public static double p = 0.05, i = 0, d = 0.0005;
    public static double f = 0.00004;

    public static final double TICKS_PER_INCH = 85.1409747739; // <== Replace with your real value!
    private ElapsedTime runtime = new ElapsedTime();

    // The tolerance we allow for the final position
    private static final double POSITION_TOLERANCE = 10; // example: 10 ticks

    //Horizontal Misumis / INTAKE
    public DcMotorEx intakeDCMotor = null;;
    public Diffy diffy = null;

    // Store the target in a class-level variable
    public double targetPosition = 0.0;

    //Circle on Sony (or B on Logitech) toggles between IntakeSlidesPosition.SHORT and IntakeSlidesPosition.FULL
    private IntakeSlidesPosition intakeSlidesPosition = IntakeSlidesPosition.CLOSE;

    private boolean usePID = false;

    public Intake(HardwareMap hardwareMap) {
        intakeDCMotor = hardwareMap.get(DcMotorEx.class, "intakedc"); //chub 0
        intakeDCMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        diffy = new Diffy(hardwareMap);

        //Following three lines required if not using PID
        if (!usePID) {
            intakeDCMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeDCMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeDCMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } else {
            controller = new PIDFController(p, i, d, f);
            controller.setTolerance(5); // optional: how close to setpoint you want to be in ticks
        }
    }

    public void initialize() {
        setPositionInInchesSync(0);
        diffy.initialize();
        diffy.openClaw();
        loop();
    }

    // 1) Method to set the PID controller’s setpoint
    public void setPositionInInches(double inches) {
        targetPosition = inches * TICKS_PER_INCH;
    }

    public void setPositionInInchesSync(double inches) {
        targetPosition = inches * TICKS_PER_INCH;
        // Run until at setpoint or forced out of loop
        while (!isAtSetpoint(intakeDCMotor.getCurrentPosition(), targetPosition)) {
            updateIntakePID();
            // Let the system keep breathing
            try {
                sleep(10);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }

        // Stop
        intakeDCMotor.setPower(0);
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

    //Called from, DriverControl:runOpMode
    public void loop() {
        if (usePID) {
            updateIntakePID();
        }
    }

    public IntakeSlidesPosition getIntakeSlidesPosition() {
        return intakeSlidesPosition;
    }

    public void setIntakeSlidesPosition(IntakeSlidesPosition position) {
        this.intakeSlidesPosition = position;
        setPositionInInches(position.getPosition());
    }

    public boolean areSlidesAtPosition() {
        return isAtSetpoint(intakeDCMotor.getCurrentPosition(), targetPosition);
    }

    public void setIntakeSlidesPositionSync(IntakeSlidesPosition position) {
        setIntakeSlidesPosition(position);
        if (!isAtSetpoint(intakeDCMotor.getCurrentPosition(), targetPosition)) {
            if (usePID) {
                setPositionInInchesSync(position.getPosition());
            } else {
                driveToPosition(1, position.getPosition(), 10000); //@ToDo
            }
        }
    }

    //If not using PID
    private void driveToPosition(double maxSpeed, double inches, double timeoutMilliSeconds) {
        // Ensure that the OpMode is still active
        int newTarget = (int)(inches * TICKS_PER_INCH);
        intakeDCMotor.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        intakeDCMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        runtime.reset();
        intakeDCMotor.setPower(Math.abs(maxSpeed));

        // keep looping while we are still active, and there is time left, and intakeDC motor is running.
        // Note: We use (isBusy()) in the loop test, which means that when intakeDC motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        while ((runtime.milliseconds() < timeoutMilliSeconds) && (intakeDCMotor.isBusy())) {
            // Set motor power
            intakeDCMotor.setPower(maxSpeed);
        }
        // Stop all motion;
        intakeDCMotor.setPower(0);
    }

}
