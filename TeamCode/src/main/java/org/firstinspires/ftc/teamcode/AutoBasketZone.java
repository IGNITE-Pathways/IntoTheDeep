package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto Basket Zone", group = "Linear OpMode")
public class AutoBasketZone extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DigitalChannel redLED;
    private DigitalChannel greenLED;
    SequentialAction sequence1 = null;

    @Override
    public void runOpMode() {

        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "rightback"); //rightback
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "rightfront"); //rightfront`
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "leftfront"); //leftfront
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "leftback"); //leftback

        redLED = hardwareMap.get(DigitalChannel.class, "redled");//7
        greenLED = hardwareMap.get(DigitalChannel.class, "greenled");//6

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        double startingXPosition = 8;
        double startingYPosition = 62;
        double moveRobotByInches = 26.5;
        double firstSpecimenXPosition = 8;

        Pose2d beginPose = new Pose2d(startingXPosition, startingYPosition, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        Viper viper = new Viper(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        viper.initialize();
        extendo.initialize();

        Action moveToDropFirstSpecimen = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches))
                .build();

        Action moveTowardsYellowSample =
                drive.actionBuilder(new Pose2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90))) //35
                        .splineToLinearHeading(new Pose2d(startingXPosition-1, startingYPosition - moveRobotByInches + 5, Math.toRadians(170)), Math.toRadians(-90)) // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                        .strafeTo(new Vector2d(21,startingYPosition - moveRobotByInches)) // moves in the direction of the sample and extendo extends
                        .build();

        sequence1 = new SequentialAction(
                viper.getReadyToDropSpecimen(),
                moveToDropFirstSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw(),
                viper.driveToPositionInInches(XBot.VIPER_HOME),
                moveTowardsYellowSample);

        // Sample Intake - Extend, move, stop, move extendo up, move elbow vertical
        SequentialAction intakeSequence = new SequentialAction(extendo.extend(), new SleepAction(0.5),
                drive.actionBuilder(new Pose2d(21, startingYPosition - moveRobotByInches, Math.toRadians(170))) // moves away from the submersible
                        .strafeTo(new Vector2d(23.5, startingYPosition - moveRobotByInches)) //@todo: TUNE
                        .waitSeconds(2)
                        .build(),
                        extendo.elbowMin()
                );

        //Spline to drop Sample to bucket
        Action driveTowardsBucket =
                drive.actionBuilder(new Pose2d(23.5, startingYPosition - moveRobotByInches, Math.toRadians(165)))
                        .splineToLinearHeading(new Pose2d(55, 50.5, Math.toRadians(43)), Math.toRadians(90)) // splines goes drop first sample in the high basket!
                        .build();


        SequentialAction takeInSampleThenDriveTowardsBucket = new SequentialAction(
                intakeSequence,
                driveTowardsBucket);

        //Move viper up while positioning
        SequentialAction dropSample = new SequentialAction(
                takeInSampleThenDriveTowardsBucket,
                extendo.elbowVertical(),
                new SleepAction(0.5),
                viper.driveToPositionInInches(XBot.VIPER_DROP_SAMPLE_HIGHER_BUCKET - 2));

        Action goaLittleBackCuzTheClawMayHitTheBasket =
                drive.actionBuilder(new Pose2d(55, 50.5, Math.toRadians(43)))
                        .strafeTo(new Vector2d(46, 45))
                        .build();

        //Drop sample to high basket
        SequentialAction sequence2 = new SequentialAction(
                dropSample,
                viper.driveToPositionInInches(XBot.VIPER_DROP_SAMPLE_HIGHER_BUCKET), //DROP
                new SleepAction(1),
                goaLittleBackCuzTheClawMayHitTheBasket,
                viper.driveToPositionInInches(XBot.VIPER_LEVEL1ASCENT)
        );

        SequentialAction goPark = new SequentialAction(
                drive.actionBuilder(new Pose2d(46, 45, Math.toRadians(45)))
                        .splineToLinearHeading(new Pose2d(45, 9, Math.toRadians(180)), Math.toRadians(270)) // splines down to get ready to park
                        .strafeTo(new Vector2d(21,9))  // splines to rung for level 1 ascent (3 points)
                        .build()
        );

        waitForStart();
        runtime.reset();

        Actions.runBlocking(sequence1);
        telemetry.addData("Sequence1 Time Used", runtime.seconds());

        Actions.runBlocking(sequence2); //TAG1
        telemetry.addData("Time Used", runtime.seconds());

        Actions.runBlocking(new SequentialAction(goPark, new SleepAction(1), viper.driveToPositionInInches(XBot.VIPER_LEVEL1ASCENT + 2)));
        // Telemetry
        telemetry.addData("Time Used", runtime.seconds());

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

}



