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
        DigitalChannel redLED = hardwareMap.get(DigitalChannel.class, "redled"); //7

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        greenLED.setMode(DigitalChannel.Mode.OUTPUT);
        redLED.setMode(DigitalChannel.Mode.OUTPUT);

        Viper viper = new Viper(hardwareMap);
        Extendo extendo = new Extendo(hardwareMap);
        viper.initialize();
        extendo.initialize();
        greenLED.setState(true); //SPECIMEN ONLY
        redLED.setState(false);

        // GAME FIELD CAN BE DIFFERENT BECAUSE OF HOW IT IS ASSEMBLED OR HOW THE MATS ARE PLACED / CUT
        double startingXPosition = -8;
        double startingYPosition = 62;
        double moveRobotByInches = 26.5;

        Pose2d beginPose = new Pose2d(startingXPosition, startingYPosition, Math.toRadians(-90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        // DEFINE X POSITIONS
        double firstSpecimenXPosition = -2;
        double secondSpecimenXPosition = -5;
        double thirdSpecimenXPosition = -6;
        double obsZoneXPosition = -44;
        double obsZoneSpecimenPickupYPositionError = 2;

        Action moveToDropFirstSpecimen = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches))
                .build();

        Action dragSampleFromSpikeMarkToObsZone =
                drive.actionBuilder(new Pose2d(firstSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90))) //35
                        .strafeTo(new Vector2d(-34, startingYPosition - moveRobotByInches + 4)) //Y=39
                        .strafeTo((new Vector2d(-34, startingYPosition - moveRobotByInches - 15))) //Y=20
                        .splineToLinearHeading(new Pose2d(obsZoneXPosition, startingYPosition - 52, Math.toRadians(90)), Math.toRadians(90)) //Y=10
                        .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 9)) //Y=53
                        .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 16)) //Y=46
                        .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError - 0.5)) //Y=60 >> Moving 50 inches straight
                        .build();

        SequentialAction sequence1 = new SequentialAction(
                viper.getReadyToDropSpecimen(),
                moveToDropFirstSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw(),
                viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN),
                dragSampleFromSpikeMarkToObsZone);

        Action moveToDropSecondSpecimen1 = drive.actionBuilder(new Pose2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError - 0.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                .build();

        Action moveToDropSecondSpecimen2 = drive.actionBuilder(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)))
                .strafeTo(new Vector2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches)) //Y=35
                .build();

        Action splineToPickThirdSpecimenFromObservationZone = drive.actionBuilder(new Pose2d(secondSpecimenXPosition, startingYPosition - moveRobotByInches, Math.toRadians(-90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(obsZoneXPosition, startingYPosition - 8, Math.toRadians(90)), Math.toRadians(90)) //Y=54
                //@ToDo: Tune Y position to make sure robot can grab the Specimen
                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError + 1.5))
                .build();

        SequentialAction sequence2 = new SequentialAction(
                viper.closeClaw(), new SleepAction(.5), //Grabs Specimen from the wall
                viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN + 4),
                new SleepAction(.2),
                moveToDropSecondSpecimen1,
                viper.getReadyToDropSpecimen(),
                moveToDropSecondSpecimen2,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw(),
                viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN),
                splineToPickThirdSpecimenFromObservationZone);

        Action moveToDropThirdSpecimen = drive.actionBuilder(new Pose2d(obsZoneXPosition, startingYPosition - obsZoneSpecimenPickupYPositionError + 1.5, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches + 2, Math.toRadians(-90)), Math.toRadians(-90)) //Y=37
                //@ToDo: Ideally the Robot should come back to original (startingYPosition - moveRobotByInches) position, but if not, we tune this
                .afterDisp(.1, viper.getReadyToDropSpecimen())
                .strafeTo(new Vector2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches - 1.5)) //Y=35
                .build();

        SequentialAction sequence3 = new SequentialAction(
                viper.closeClaw(), new SleepAction(.5), //Grabs Specimen from the wall
                viper.driveToPositionInInches(XBot.VIPER_PICK_SPECIMEN + 4),
                moveToDropThirdSpecimen,
                viper.driveToPositionInInches(XBot.DROPPED_SPECIMEN),
                viper.openClaw());

        Action parkingAction = drive.actionBuilder(new Pose2d(thirdSpecimenXPosition, startingYPosition - moveRobotByInches - 1.5, Math.toRadians(-90)))
                //@ToDo: Tune Y position to make sure robot can grab the Specimen
                .strafeTo(new Vector2d(obsZoneXPosition, startingYPosition - 5)) //Y=57
                .build();

        SequentialAction sequence4 = new SequentialAction(viper.closeClaw(), viper.driveToPositionInInches(XBot.VIPER_HOME), parkingAction);

        waitForStart();
        runtime.reset();

        ///////////////////////////////////////////////////////////////////////////////

        //sequence1 = 1st Specimen dropped, 2nd Specimen in Obs Zone, Robot in Obs Zone
        Actions.runBlocking(sequence1);
        telemetry.addData("First Sequence:", runtime.seconds());

        Actions.runBlocking(viper.closeClaw());

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



