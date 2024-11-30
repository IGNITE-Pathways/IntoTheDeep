package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name = "AutoRUn", group = "Linear OpMode")
public class AutoRun extends LinearOpMode {

    boolean rolling = false;
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private final DcMotor viper = null;
    private Servo extendo = null;
    private Servo elbow = null;
    private Servo roller = null;
    private Servo claw = null;
    private DigitalChannel redLED;
    private DigitalChannel greenLED;

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "rightback"); //rightback
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "rightfront"); //rightfront`
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "leftfront"); //leftfront
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "leftback"); //leftback

        redLED = hardwareMap.get(DigitalChannel.class, "redled");//7
        greenLED = hardwareMap.get(DigitalChannel.class, "greenled");//6

        //Initialize the Servo variables
        extendo = hardwareMap.get(Servo.class, "extendo"); // chub 0
        roller = hardwareMap.get(Servo.class, "roller"); // chub 1
        elbow = hardwareMap.get(Servo.class, "elbow"); // chub 5
        claw = hardwareMap.get(Servo.class, "claw"); // ehub 3

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        initializeSystems();

        Pose2d beginPose = new Pose2d(0, 0, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, new Pose2d(12.5, 63, Math.toRadians(-90)));
        Viper viper = new Viper(hardwareMap);
        waitForStart();
        runtime.reset();

//        ParallelAction pa = new ParallelAction(
//                drive.followTrajectory(shootingTraj),
//                new SequentialAction(
//                        shooter.spinUp(),
//                        shooter.fireBall()
//                )
//        );

        SequentialAction sa = new SequentialAction(
                drive.actionBuilder(new Pose2d(12.5, 63, Math.toRadians(-90)))
                        .lineToYConstantHeading(31).build(), viper.getReadyToDropSpecimen()
        );

        Action blueRightAction = drive.actionBuilder(new Pose2d(12.5, 63, Math.toRadians(-90)))
                .lineToYConstantHeading(31) // drives to the chamber
                .afterDisp(0, viper.getReadyToDropSpecimen())
                // drops preloaded specimen on the chamber
//                .afterTime(1000, () -> {
//                    dropSpecimen();
//                })
                //go to pick next yellow sample
                .splineToLinearHeading(new Pose2d(12.5, 37, Math.toRadians(165)), Math.toRadians(-90)) // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                .strafeTo(new Vector2d(26, 32)) // moves in the direction of the sample and extendo extends
//                // extendo takes in the sample
//                .waitSeconds(1)
//                //drop the sample
//                .splineTo(new Vector2d(55, 56), Math.toRadians(45))
//                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
//                //Pick next one
//                .splineToLinearHeading(new Pose2d(38, 38, Math.toRadians(135)), Math.toRadians(-90)) // robot aligns itself to get the second sample
//                .waitSeconds(2) // extendo extends and takes in the second sample
//                //drop again
//                .splineTo(new Vector2d(55, 56), Math.toRadians(45)) // splines goes drop first sample in the high basket!
//                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
//                //Pick again
//                .splineToLinearHeading(new Pose2d(50, 40, Math.toRadians(135)), Math.toRadians(-90)) // robot aligns itself to get the second sample
//                .waitSeconds(2) // extendo extends and takes in the second sample
//                //drop again
//                .splineTo(new Vector2d(55, 56), Math.toRadians(45)) // splines goes drop first sample in the high basket!
//                .waitSeconds(3) // viper slides go up and robot drops the sample in the basket
//                .strafeTo(new Vector2d(50, 38)) // splines goes drop first sample in the high basket!
//                .splineToLinearHeading(new Pose2d(21.5, 10, Math.toRadians(0)), Math.toRadians(90)) // splines to rung for level 1 ascent (3 points)
                .build();

        Actions.runBlocking(blueRightAction);

        // Telemetry
        telemetry.addData("AutoRun", "DONE");

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

    private void initializeSystems() {
        extendo.setPosition(XBot.EXTENDO_MIN);
        elbow.setPosition(XBot.ELBOW_VERTICAL);
        claw.setPosition(XBot.CLAW_CLOSE);
    }
}



