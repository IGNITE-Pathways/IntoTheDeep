/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Roller Test", group = "Linear Opmode")
public class RollerTest extends LinearOpMode {
    private Servo roller;
    @Override
    public void runOpMode() {
        roller = hardwareMap.get(Servo.class, "roller");
        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                roller.setPosition(1.0);
            } else if (gamepad1.b) {
                roller.setPosition(0.0);
            }
            telemetry.addData("Roller Servo Position", roller.getPosition());
            telemetry.update();
        }
    }
} */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Roller Test", group = "Linear Opmode")
public class RollerTest extends LinearOpMode {
    private Servo roller;

    @Override
    public void runOpMode() {
        // Initialize the servo
        roller = hardwareMap.get(Servo.class, "roller");

        // Set the initial position of the servo to 0.5
        roller.setPosition(0.5);

        // Display the initial state on the telemetry
        telemetry.addData("Status", "Initialized to 0.5");
        telemetry.addData("Roller Servo Position", roller.getPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Set position to 1.0 when Button A is pressed
            if (gamepad1.a) {
                roller.setPosition(1.0);
            }
            // Set position to 0.0 when Button B is pressed
            else if (gamepad1.b) {
                roller.setPosition(0.0);
            }

            // Continuously display the current servo position
            telemetry.addData("Roller Servo Position", roller.getPosition());
            telemetry.update();
        }
    }
}


