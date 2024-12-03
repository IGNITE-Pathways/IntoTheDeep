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

@Autonomous(name = "Auto Obs Zone", group = "Linear OpMode")
public class AutoObservationZone extends LinearOpMode {

    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        DcMotor leftFrontDrive = hardwareMap.get(DcMotorEx.class, "rightback"); //rightback
        DcMotor leftBackDrive = hardwareMap.get(DcMotorEx.class, "rightfront"); //rightfront`
        DcMotor rightBackDrive = hardwareMap.get(DcMotorEx.class, "leftfront"); //leftfront
        DcMotor rightFrontDrive = hardwareMap.get(DcMotorEx.class, "leftback"); //leftback
        DigitalChannel greenLED = hardwareMap.get(DigitalChannel.class, "greenled"); //6

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        greenLED.setMode(DigitalChannel.Mode.OUTPUT);

        Viper viper = new Viper(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        viper.initialize();
        extendo.initialize();
        greenLED.setState(true); //SPECIMEN ONLY

        // GAME FIELD CAN BE DIFFERENT BECAUSE OF HOW IT IS ASSEMBLED OR HOW THE MATS ARE PLACED / CUT
        int startingYPosition = 62;
        int moveRobotByInches = 27;

        Pose2d beginPose = new Pose2d(-8, startingYPosition, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        // DEFINE X POSITIONS
        int firstSpecimenXPosition = -2;
        int secondSpecimenXPosition = -6;
        int thirdSpecimenXPosition = -10;
        int parkingXPosition = -44;

        Action moveToDropFirstSpecimen = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches))
                .build();

        Action dragSampleFromSpikeMarkToObsZone =
                drive.actionBuilder(new Pose2d(-2, startingYPosition - moveRobotByInches, Math.toRadians(-90))) //35
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-34, startingYPosition - moveRobotByInches - 7, Math.toRadians(0)), Math.toRadians(-90)) //Y=29
                        .splineToLinearHeading(new Pose2d(-46, startingYPosition - 52, Math.toRadians(90)), Math.toRadians(90)) //Y=10
                        //@ToDo: Tune Y position to make sure robot can grab the Specimen
                        .strafeTo(new Vector2d(-46, startingYPosition - 1)) //Y=61 >> Moving 51 inches straight
                        .afterDisp(2,viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN)) //Should happen in parallel
                        .waitSeconds(.2)
                        .build();

        SequentialAction sequence1 = new SequentialAction(
                viper.getReadyToDropSpecimen(),
                moveToDropFirstSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw(),
                viper.driveToPositionInInches(XBot.VIPER_HOME),
                dragSampleFromSpikeMarkToObsZone);

        Action moveToDropSecondSpecimen = drive.actionBuilder(new Pose2d(-46, startingYPosition - 1, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                //@ToDo: Ideally the Robot should come back to original (startingYPosition - moveRobotByInches) position, but if not, we tune this
                .strafeTo(new Vector2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .build();

        Action splineToPickThirdSpecimenFromObservationZone = drive.actionBuilder(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-46, startingYPosition - 8, Math.toRadians(90)), Math.toRadians(90)) //Y=54
                //@ToDo: Tune Y position to make sure robot can grab the Specimen
                .strafeTo(new Vector2d(-46, startingYPosition - 1))
                .build();

        SequentialAction sequence2 = new SequentialAction(
                viper.closeClaw(), new SleepAction(.2), //Grabs Specimen from the wall
                viper.getReadyToDropSpecimen(),
                moveToDropSecondSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw(),
                viper.driveToPositionInInches(XBot.VIPER_HOME),
                splineToPickThirdSpecimenFromObservationZone);

        Action moveToDropThirdSpecimen = drive.actionBuilder(new Pose2d(-46, startingYPosition - 1, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                //@ToDo: Ideally the Robot should come back to original (startingYPosition - moveRobotByInches) position, but if not, we tune this
                .strafeTo(new Vector2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .build();

        SequentialAction sequence3 = new SequentialAction(
                viper.closeClaw(), new SleepAction(.2), //Grabs Specimen from the wall
                viper.getReadyToDropSpecimen(),
                moveToDropThirdSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw());

        Action parkingAction = drive.actionBuilder(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90)))
                //@ToDo: Tune Y position to make sure robot can grab the Specimen
                .strafeTo(new Vector2d(parkingXPosition, startingYPosition - 1 )) //Y=61
                .build();

        SequentialAction sequence4 = new SequentialAction(parkingAction, extendo.elbowVertical(),
                viper.driveToPositionInInches(XBot.VIPER_HOME));

        waitForStart();
        runtime.reset();

        ///////////////////////////////////////////////////////////////////////////////

        //sequence1 = 1st Specimen dropped, 2nd Specimen in Obs Zone, Robot in Obs Zone
        Actions.runBlocking(sequence1);
        telemetry.addData("First Sequence:", runtime.seconds());

        //sequence2 = 2nd Specimen dropped, Robot in Obs Zone
        Actions.runBlocking(sequence2);
        telemetry.addData("Second Sequence:", runtime.seconds());

        //sequence3 = 3rd Specimen dropped
        Actions.runBlocking(sequence3); //3nd Specimen dropped
        telemetry.addData("Third Sequence:", runtime.seconds());

        //sequence4 = Robot Parked
        Actions.runBlocking(sequence4);
        telemetry.addData("Forth Sequence:", runtime.seconds());

        telemetry.update();
    }

}



