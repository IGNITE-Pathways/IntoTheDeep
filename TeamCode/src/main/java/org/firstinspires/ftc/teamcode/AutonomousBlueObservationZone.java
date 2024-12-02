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

@Autonomous(name = "Auto Blue Obs Zone", group = "Linear OpMode")
public class AutonomousBlueObservationZone extends LinearOpMode {

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

        Pose2d beginPose = new Pose2d(-8, 63, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);
        Viper viper = new Viper(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        viper.initialize();
        extendo.initialize();

        waitForStart();
        runtime.reset();

        Action moveToDropSpecimenLocation = drive.actionBuilder(
                new Pose2d(-8, 63, Math.toRadians(-90)))
                .strafeTo(new Vector2d(-2, 36))
                .build();

        //Move to drop specimen while rising
        SequentialAction raiseViperWhenMovingToDropSpecimen = new SequentialAction(
                viper.getReadyToDropSpecimen(),
                moveToDropSpecimenLocation);

        // drops preloaded specimen on the chamber
        SequentialAction dropSpecimenSequence = new SequentialAction(
                raiseViperWhenMovingToDropSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw());

        Action moveToThePositionOfFIRSTSample =
                drive.actionBuilder(new Pose2d(-8, 36, Math.toRadians(-90)))
                        .strafeToConstantHeading(new Vector2d(-33, 40)) // drives to the left
                        .strafeToConstantHeading((new Vector2d(-35, 35))) // gets ready to do a nice spline, without hitting the top left stand bar holding up the submersible
                        .splineToLinearHeading(new Pose2d(-46, 10, Math.toRadians(180)), Math.toRadians(90))
                        .build();

        //Move the viper slide down while moving to the position to pick next sample
        SequentialAction moveViperDownWhenMovingToTheFIRSTSample = new SequentialAction(
                viper.driveToPositionInInches(XBot.VIPER_HOME),
                moveToThePositionOfFIRSTSample);


        Action pushTheSampleIntoTheObservationZone =
                drive.actionBuilder(new Pose2d(-46, 10, Math.toRadians(180)))
                        .strafeTo(new Vector2d(-46, 52)) // pushes 1st sample into to the observation zone
                        .build();

        SequentialAction SequenceOfActions = new SequentialAction(
                dropSpecimenSequence,
                moveViperDownWhenMovingToTheFIRSTSample,
                pushTheSampleIntoTheObservationZone);

        Actions.runBlocking(SequenceOfActions);
        telemetry.addData("Time Used", runtime.seconds());

        Action pushSecondSampleToObsZone = drive.actionBuilder(new Pose2d(-46, 52, Math.toRadians(180)))
                .strafeTo(new Vector2d(-45, 35)) // strafes a little down and right to make a smooth spline
                .splineToConstantHeading(new Vector2d(-55, 16), Math.toRadians(180))  // goes to second sample
                .strafeTo(new Vector2d(-55, 52)) // pushes 2nd sample into the observation zone
                .build();

        Actions.runBlocking(pushSecondSampleToObsZone);

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



