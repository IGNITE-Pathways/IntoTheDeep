package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Tele Op", group="Linear OpMode")
public class DriverControl extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private double robotSpeed = 1.0;

    Intake intake = null;
    Outtake outtake = null;

    private void initializeSystems() {
        intake.initialize();
        outtake.initialize();
        gameState = GameState.INIT;
        telemetry.addData("Initialized", "Done");
        telemetry.update();
    }

    private enum GameElement {
        SAMPLE,
        SPECIMEN,
        SPECIMEN_TO_BE
    }
    private GameElement gameElement = GameElement.SAMPLE; // Default no state

    private enum GameState {
        //Get into state: driver hits Init on Driver station, also default state after AUTO-OP
        //What happens in state? Nothing. Robot should already have DiffyVerticalPosition is FLAT, claw open,
        // h-slides at transfer position, v-slides at transfer position, OuttakeArmPosition at FACING_DOWN
        //Get out of state: Driver hits Play button on Driver station, state changes to PICKING_GAME_ELEMENT
        INIT,

        //Get into state: driver hits play on Driver station,
        //What happens in state? DiffyVerticalPosition is FLAT or DOWN, PickingPosition is FULL or SHORT, OuttakeArmPosition start moving to TRANSFER
        // Driver can rotate claw (left / right bumpers), Limelight sensor can auto-rotate claw
        //Get out of state: driver2 hits circle (or B) button when ready to pick game element,
        // or square (or X) button when ready to pick Specimen, state changes to GAME_ELEMENT_IN_INTAKE_CLAW
        PICKING_GAME_ELEMENT,

        //Get into state: when in PICKING_GAME_ELEMENT mode, driver hits circle (Sony) or B (Logitech) button on GamePad 2 to pick game element
        //What happens in state? Intake Claw closes to pick / hold the game element, intake and outtake moves to transfer position
        //Get out of state: auto-change to TRANSFERRING_GAME_ELEMENT
        GAME_ELEMENT_IN_INTAKE_CLAW,

        //Get into state: auto-change from GAME_ELEMENT_IN_INTAKE_CLAW
        //What happens in state? transfer happens, outtake claw closes, intake claw opens
        //Get out of state: auto-change to GAME_ELEMENT_IN_OUTTAKE_CLAW after auto-transfer
        TRANSFERRING_GAME_ELEMENT,

        //Get into state: auto-change from TRANSFERRING_GAME_ELEMENT after auto-transfer
        //What happens in state? GameElement transfers to outtake claw, the intake claw opens, diffy goes flat
        //Get out of state: auto-change to GOING_TO_DROP_GAME_ELEMENT after moving intake back
        GAME_ELEMENT_IN_OUTTAKE_CLAW,

        //Get into state: auto-change from GAME_ELEMENT_IN_OUTTAKE_CLAW
        //What happens in state? If GameElement is Sample, OuttakeSlidesPosition move to DROP_SAMPLE, OuttakeArmPosition change to SAMPLE_DROP
        // If GameElement is Specimen, OuttakeSlidesPosition move to HOOK_SPECIMEN_TOP_RUNG, OuttakeArmPosition change to SPECIMEN_DROP
        //Get out of state: auto-change to READY_TO_DROP_GAME_ELEMENT after v-slides and arm reached correct positions
        GOING_TO_DROP_GAME_ELEMENT,

        //Get into state: auto-change from GOING_TO_DROP_GAME_ELEMENT
        //What happens in state? Allows Driver to move, align robot and get ready to drop GameElement
        //Get out of this state: Drive and drop the GameElement, state change to DROPPED_GAME_ELEMENT after user-action
        READY_TO_DROP_GAME_ELEMENT,

        //Get into state: auto-change from READY_TO_DROP_GAME_ELEMENT when user takes drop action
        //What happens in state? GameElement (Sample or Specimen) is already dropped, intake claw opens, DiffyVerticalPosition FLAT,
        // OuttakeSlidesPosition move to TRANSFER, OuttakeArmPosition move to TRANSFER
        //Get into state: auto-change to PICKING_GAME_ELEMENT
        DROPPED_GAME_ELEMENT
    }

    private GameState gameState = GameState.INIT;

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

        gameState = GameState.INIT;
        initializeSystems();

        telemetry.addData("Status", gameState);
        telemetry.addData("COUNTS_PER_INCH", XBot.COUNTS_PER_INCH);
        telemetry.addData("Outtake Motor Pos", "%7d: %7d", outtake.getLeftPosition(), outtake.getRightPosition());
        telemetry.update();

        waitForStart();
        runtime.reset();
        //Play Button Pressed
        gameState = GameState.PICKING_GAME_ELEMENT;

        double lastDiffyDegreesChanged = runtime.milliseconds();
        double lastDiffyAngleChanged = runtime.milliseconds();
        // run until the end of the match (driver presses STOP)
        double leftFrontPower = 0;
        double leftBackPower = 0;
        double rightFrontPower = 0;
        double rightBackPower = 0;

        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            leftFrontPower = axial + lateral + yaw;
            rightFrontPower = axial - lateral - yaw;
            leftBackPower = axial - lateral + yaw;
            rightBackPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

//            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
//            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
//            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
//            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * robotSpeed);
            rightFrontDrive.setPower(rightFrontPower * robotSpeed);
            leftBackDrive.setPower(leftBackPower * robotSpeed);
            rightBackDrive.setPower(rightBackPower * robotSpeed);

            // TELE-OP KEY-BINDS / ACTIONS
            switch (gameState) {
                case INIT:
                    initializeSystems();
                    break;
                case PICKING_GAME_ELEMENT:
                    intake.diffy.setIntakeClawPosition(ClawPosition.OPEN);
                    if (gamepad2.dpad_down) {
                        //FLIP
                        intake.diffy.setDiffyPosition(DiffyPosition.DOWN_PARALLEL);
                    } else if (gamepad2.dpad_up) {
                        intake.diffy.setDiffyPosition(DiffyPosition.FLAT);
                    }

                    if (gamepad2.dpad_right) {
                        intake.setIntakeSlidesPositionSync(IntakeSlidesPosition.FULL);
                    } else if (gamepad2.dpad_left) {
                        intake.setIntakeSlidesPositionSync(IntakeSlidesPosition.SHORT);
                    }

                    outtake.setOuttakeSlidesPositionSync(OuttakeSlidesPosition.TRANSFER);
                    outtake.setOuttakeArmPosition(OuttakeArmPosition.TRANSFER);
                    outtake.setOuttakeClawPosition(ClawPosition.OPEN);
                    if (gamepad2.triangle) {
                        //Action to PICK SAMPLE
                        gameElement = GameElement.SAMPLE;
                        intake.diffy.setIntakeClawPosition(ClawPosition.CLOSE);
                        gameState = GameState.GAME_ELEMENT_IN_INTAKE_CLAW;
                    } else if (gamepad2.square) {
                        //Action to PICK SPECIMEN
                        if (intake.diffy.getSampleColor() == SampleColor.YELLOW) {
                            gameElement = GameElement.SAMPLE;
                            intake.diffy.setIntakeClawPosition(ClawPosition.CLOSE);
                            gameState = GameState.GAME_ELEMENT_IN_INTAKE_CLAW;
                        } else {
                            gameElement = GameElement.SPECIMEN;
                            intake.diffy.setIntakeClawPosition(ClawPosition.CLOSE);
                            gameState = GameState.GAME_ELEMENT_IN_INTAKE_CLAW;
                        }
                    } else if (gamepad2.cross) {
                        if (intake.diffy.getSampleColor() == SampleColor.YELLOW) {
                            gameElement = GameElement.SAMPLE;
                            intake.diffy.setIntakeClawPosition(ClawPosition.CLOSE);
                            gameState = GameState.GAME_ELEMENT_IN_INTAKE_CLAW;
                        } else {
                            gameElement = GameElement.SPECIMEN_TO_BE;
                            intake.diffy.setIntakeClawPosition(ClawPosition.CLOSE);
                            gameState = GameState.GOING_TO_DROP_GAME_ELEMENT; //Directly jump to DROP Game Element
                        }
                    }
                    //Implement gamepad2.right_stick_x to rotate diffy
                    if ((Math.abs(gamepad2.right_stick_x) >= 0.5) && ((runtime.milliseconds() - lastDiffyDegreesChanged) > 200)) {
                        int sign = (gamepad2.right_stick_x == 0) ? 0 : (gamepad2.right_stick_x > 0) ? 1 : -1;
                        intake.diffy.cycleDownPosition(sign);
                        lastDiffyDegreesChanged = runtime.milliseconds();
                    }
                    break;
                case GAME_ELEMENT_IN_INTAKE_CLAW:
                    if (intake.diffy.isClawClosed()) {
                        //intake and outtake moves to transfer position
                        intake.setIntakeSlidesPositionSync(IntakeSlidesPosition.TRANSFER);
                        intake.diffy.setDiffyPosition(DiffyPosition.TRANSFER_SAMPLE); //@ToDo:
                        intake.diffy.setIntakeClawPosition(ClawPosition.CLOSE);
                        outtake.setOuttakeSlidesPositionSync(OuttakeSlidesPosition.TRANSFER); //Short distance, go sync
                        outtake.setOuttakeArmPosition(OuttakeArmPosition.TRANSFER);
                        outtake.setOuttakeClawPosition(ClawPosition.OPEN);
                    }
                    break;
                case TRANSFERRING_GAME_ELEMENT:
                    //if intake and outtake are aligned
                    //transfer happens, outtake claw closes, intake claw opens
                    outtake.setOuttakeClawPosition(ClawPosition.CLOSE);
                    gameState = GameState.GAME_ELEMENT_IN_OUTTAKE_CLAW;
                    break;
                case GAME_ELEMENT_IN_OUTTAKE_CLAW:
                    if (outtake.isClawClosed()) {
                        //the intake claw opens, diffy goes flat
                        intake.diffy.setIntakeClawPosition(ClawPosition.OPEN);
                        intake.diffy.setDiffyPosition(DiffyPosition.FLAT);
                        gameState = GameState.GOING_TO_DROP_GAME_ELEMENT;
                    }
                    break;
                case GOING_TO_DROP_GAME_ELEMENT:
                    //If GameElement is Sample, OuttakeSlidesPosition move to DROP_SAMPLE, OuttakeArmPosition change to SAMPLE_DROP
                    //If GameElement is Specimen, OuttakeSlidesPosition move to HOOK_SPECIMEN_TOP_RUNG, OuttakeArmPosition change to SPECIMEN_DROP
                    switch (gameElement) {
                        case SAMPLE:
                            outtake.setOuttakeSlidesPosition(OuttakeSlidesPosition.DROP_SAMPLE);
                            outtake.setOuttakeArmPosition(OuttakeArmPosition.SAMPLE_DROP);
                            break;
                        case SPECIMEN:
                            outtake.setOuttakeSlidesPosition(OuttakeSlidesPosition.HOOK_SPECIMEN_TOP_RUNG);
                            outtake.setOuttakeArmPosition(OuttakeArmPosition.SPECIMEN_DROP);
                            break;
                        case SPECIMEN_TO_BE:
                            intake.diffy.setDiffyPosition(DiffyPosition.FLAT);
                            intake.setIntakeSlidesPositionSync(IntakeSlidesPosition.SHORT); //@ToDo: Do we need this
                            break;
                    }
                    break;
                case READY_TO_DROP_GAME_ELEMENT:
                    // Allows Driver to move, align robot and get ready to drop GameElement
                    if (gamepad2.dpad_up) {
                        switch (gameElement) {
                            case SAMPLE:
                                outtake.setOuttakeClawPosition(ClawPosition.OPEN);
                                break;
                            case SPECIMEN:
                                outtake.setOuttakeClawPosition(ClawPosition.OPEN);
                                break;
                            case SPECIMEN_TO_BE:
                                intake.diffy.setIntakeClawPosition(ClawPosition.OPEN);
                                outtake.setOuttakeClawPosition(ClawPosition.OPEN);
                                break;
                        }
                        gameState = GameState.DROPPED_GAME_ELEMENT;
                    }
                    break;
                case DROPPED_GAME_ELEMENT:
                    if (outtake.isClawOpen() && intake.diffy.isClawOpen()) {
                        intake.diffy.setDiffyPosition(DiffyPosition.FLAT);
                        gameState = GameState.PICKING_GAME_ELEMENT;
                    }
                    break;
            }

            // Always update the PID each loop
            intake.loop();
            outtake.loop();

            // Telemetry
            telemetry.addData("GAME Element", gameElement);
            telemetry.addData("GAME State", gameState);
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            //OUTTAKE
            telemetry.addData("Outtake Motor Pos:", "%7d: %7d", outtake.getLeftPosition(), outtake.getRightPosition());
            telemetry.addData("Outtake Target Pos:", outtake.targetSlidesPosition);

            //INTAKE
            telemetry.addData("Diffy Position, Left:", "%4.2f, Right: %4.2f", intake.diffy.diffyLeft.getPosition(), intake.diffy.diffyRight.getPosition());
            telemetry.addData("Intake Sensor, Color: " + intake.diffy.getSampleColor(), "Distance: %4.2f", intake.diffy.intakeSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("Intake Motor Pos: ", "%7d", intake.getPosition());
            telemetry.addData("Intake Slides position: ", intake.getIntakeSlidesPosition());

            //MOTORS
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
            sleep(1);
        }
    }
}
