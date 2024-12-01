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

@Autonomous(name = "AutoRun", group = "Linear OpMode")
public class AutonomousBlueBasket extends LinearOpMode {

    boolean rolling = false;
    // Declare OpMode members for each of the 4 motors.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

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

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // change LED mode from input to output
        redLED.setMode(DigitalChannel.Mode.OUTPUT);
        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        Pose2d beginPose = new Pose2d(8, 63, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Viper viper = new Viper(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        viper.initialize();
        extendo.initialize();

        waitForStart();
        runtime.reset();

        Action moveToDropSpecimenLocation = drive.actionBuilder(new Pose2d(8, 63, Math.toRadians(-90)))
                .lineToYConstantHeading(34).build();

        //Move to drop specimen while rising
        SequentialAction raiseViperWhenMovingToDropSpecimen = new SequentialAction(viper.getReadyToDropSpecimen(), moveToDropSpecimenLocation);

        // drops preloaded specimen on the chamber
        SequentialAction dropSpecimenSequence = new SequentialAction(
                raiseViperWhenMovingToDropSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw());

//        Actions.runBlocking(dropSpecimenSequence);

        Action moveToPickSample =
                drive.actionBuilder(new Pose2d(8, 34, Math.toRadians(-90)))
                //go to pick next yellow sample
                .splineToLinearHeading(new Pose2d(8, 40, Math.toRadians(165)), Math.toRadians(-90)) // goes back so it doesn't hit the hitting the top right stand bar holding up the submersible
                .strafeTo(new Vector2d(22, 35)) // moves in the direction of the sample and extendo extends
                .build();

        //Move the viper slide down while moving to the position to pick next sample
        SequentialAction moveViperDownWhenMovingToPickSample = new SequentialAction(
                viper.driveToPositionInInches(XBot.VIPER_HOME), moveToPickSample);

        SequentialAction readyToPickSampleSequence = new SequentialAction(
                dropSpecimenSequence,
                moveViperDownWhenMovingToPickSample);

        Actions.runBlocking(readyToPickSampleSequence);

        telemetry.addData("Time Used", runtime.seconds());

        // Sample Intake - Extend, move, stop, move extendo up, move elbow vertical
        SequentialAction intakeSequence = new SequentialAction(extendo.extend(),

                drive.actionBuilder(new Pose2d(22, 35, Math.toRadians(165))) // moves away from the submersible
                        .strafeTo(new Vector2d(23, 33)) //@todo: TUNE
                        .waitSeconds(2)
                        .build(),
                        extendo.elbowMin()
                );

        //Spline to drop Sample to bucket
        Action driveTowardsBucket =
                drive.actionBuilder(new Pose2d(23, 33, Math.toRadians(165)))
                        .splineTo(new Vector2d(50, 49), Math.toRadians(45))
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

        //Drop sample to high basket
        SequentialAction dropSampleSequence = new SequentialAction(dropSample,
                viper.driveToPositionInInches(XBot.VIPER_DROP_SAMPLE_HIGHER_BUCKET), new SleepAction(1)
        );

        SequentialAction gotoSecondSample = new SequentialAction(viper.driveToPositionInInches(XBot.VIPER_HOME),
                drive.actionBuilder(new Pose2d(50, 49, Math.toRadians(45)))
                        .splineToLinearHeading(new Pose2d(38, 38, Math.toRadians(135)), Math.toRadians(-90))
                        .build()
        );

//        SequentialAction gotoThirdSample = new SequentialAction(viper.driveToPositionInInches(XBot.VIPER_HOME),
//                drive.actionBuilder(new Pose2d(38, 38, Math.toRadians(170)))
//                        .splineToLinearHeading(new Pose2d(38, 38, Math.toRadians(135)), Math.toRadians(-90))
//                        .build()
//        );

        SequentialAction dropTheFirstSampleThenMoveToSecondSampleAndRepeat = new SequentialAction(dropSample,
                gotoSecondSample,
                takeInSampleThenDriveTowardsBucket);


        Actions.runBlocking(dropTheFirstSampleThenMoveToSecondSampleAndRepeat);
        telemetry.addData("Time Used", runtime.seconds());

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

        Action parkingAction = drive.actionBuilder(new Pose2d(55, 56, Math.toRadians(45)))
                .strafeTo(new Vector2d(50, 38)) // splines goes drop first sample in the high basket!
                .splineToLinearHeading(new Pose2d(21.5, 10, Math.toRadians(0)), Math.toRadians(90)) // splines to rung for level 1 ascent (3 points)
                .build();

//        Actions.runBlocking(parkingAction);

        // Telemetry
        telemetry.addData("Time Used", runtime.seconds());

        // Show the elapsed game time and wheel power.
        telemetry.update();
    }

}



